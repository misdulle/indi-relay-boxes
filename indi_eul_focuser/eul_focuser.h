/*
 * INDI driver: closed-loop stepper focuser for Raspberry Pi 5
 *   - STEP / DIR / ENABLE outputs via the kernel GPIO character device
 *   - position feedback from a UDP stream (Arduino + mechanical encoder)
 *
 * Units: everything user-facing is in micrometres (um).
 */
#pragma once

#include <indifocuser.h>
#include <indiapi.h>

#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <cstdint>

#include "gpio_line.h"

// FocuserInterface switched from legacy C structs (FocusAbsPosNP/FocusAbsPosN)
// to INDI::PropertyNumber wrappers during the 2.0.x series. If compilation
// fails around FocusAbsPosNP on your build, flip this switch.
#ifndef EF_FOCUS_WRAPPED
#  if (INDI_VERSION_MAJOR > 2) || (INDI_VERSION_MAJOR == 2 && (INDI_VERSION_MINOR > 0 || INDI_VERSION_RELEASE >= 7))
#    define EF_FOCUS_WRAPPED 1
#  else
#    define EF_FOCUS_WRAPPED 0
#  endif
#endif

class EulFocuser : public INDI::Focuser
{
    public:
        EulFocuser();
        ~EulFocuser() override;

        const char *getDefaultName() override;

        bool initProperties() override;
        void ISGetProperties(const char *dev) override;
        bool updateProperties() override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

    protected:
        bool Connect() override;
        bool Disconnect() override;
        void TimerHit() override;
        bool saveConfigItems(FILE *fp) override;

        IPState MoveAbsFocuser(uint32_t targetTicks) override;
        IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
        bool AbortFocuser() override;
        bool SyncFocuser(uint32_t ticks) override;
        bool SetFocuserMaxPosition(uint32_t ticks) override;

    private:
        // ---- configuration snapshot used by the motion thread -------------------------
        struct Cfg
        {
            double ustepsPerMm = 1900, delayFast = 125, delayFine = 400, delayStart = 1000;
            double rampSteps = 0, fineZoneUm = 100, pulseUs = 5;
            double tolUm = 1, backIn = 500, backOut = 500, stallMarginUm = 300;
            int maxPasses = 5;
            int minPos = 500, maxPos = 6000;
            int staleMovingMs = 500;
            bool enableActiveLow = true, dirHighIncreases = true;
            int rtPrio = 50;
        };

        enum class MoveResult : int { Ok = 0, Aborted, Stall, StaleData, MaxPasses, Limit, GpioError };

        // ---- helpers ---------------------------------------------------------------------
        void refreshCfg();
        bool startUdp(std::string &err);
        void stopUdp();
        void udpLoop();
        void parseDatagram(const char *buf, size_t len);

        int  positionUm() const;
        int64_t dataAgeMs() const;
        void setEnable(bool on, bool activeLow);
        void stopMotion();
        IPState startMove(int targetUm);
        void motionThread(int targetUm, Cfg c);
        void finishMove();
        void persistPosition();

        // compat accessors for the Focuser base class properties
        void setAbsValue(double v);
        void setAbsState(IPState s);
        void applyAbs();
        void setRelState(IPState s);
        void applyRel();
        void setAbsLimits(double mn, double mx);
        double getMaxPos();
        void setMaxPos(double v);

        static int64_t nowNs();
        static void waitUntil(int64_t ns);

        // ---- properties (all defined at startup so they can be configured before connecting)
        INDI::PropertyNumber UdpNP     {3};   // port, stale timeouts
        INDI::PropertyNumber GpioNP    {4};   // chip, step, dir, enable
        INDI::PropertyNumber MotorNP   {9};
        INDI::PropertyNumber LoopNP    {7};
        INDI::PropertyNumber PosStateNP{2};   // persisted
        INDI::PropertyNumber StatusNP  {3};   // read only, defined when connected
        INDI::PropertySwitch EnablePolSP{2};
        INDI::PropertySwitch DirPolSP   {2};
        INDI::PropertySwitch RampSP     {2};
        INDI::PropertySwitch RestoreSP  {2};

        enum { U_PORT, U_STALE_MOVING, U_STALE_IDLE };
        enum { G_CHIP, G_STEP, G_DIR, G_ENABLE };
        enum { M_USTEPS_MM, M_USTEPS_STEP, M_DELAY_FAST, M_DELAY_FINE, M_DELAY_START, M_RAMP_STEPS,
               M_FINE_ZONE, M_PULSE, M_HOLD_MS };
        enum { L_TOL, L_PASSES, L_BACK_IN, L_BACK_OUT, L_STALL_MARGIN, L_UM_PER_COUNT, L_MIN_POS };
        enum { P_LAST, P_OFFSET };
        enum { S_RAW, S_POS, S_AGE };

        // ---- runtime state ---------------------------------------------------------------
        Cfg cfg_;
        std::mutex cfgMutex_;

        GpioOutputs gpio_;
        std::atomic<bool> motorEnabled_{false};

        // UDP
        int udpFd_ = -1;
        std::thread udpThread_;
        std::atomic<bool> udpRun_{false};
        std::atomic<int>  encRaw_{0};
        std::atomic<bool> encValid_{false};
        std::atomic<int64_t> lastRxNs_{0};
        std::atomic<int>  scaleUm_{1};
        std::atomic<int>  offsetUm_{0};

        // motion
        std::thread moveThread_;
        std::atomic<bool> abortReq_{false};
        std::atomic<bool> moving_{false};
        std::atomic<bool> moveDone_{false};
        std::atomic<int>  moveResult_{0};
        std::atomic<int>  lastMoveDir_{0};
        std::atomic<bool> rtFailed_{false};
        std::atomic<int>  moveTarget_{0};
        int64_t lastMoveEndNs_ = 0;

        bool configLoaded_ = false;
        bool restorePending_ = false;
        bool staleReported_ = false;
        bool rtReported_ = false;
        int  tick_ = 0;
        int  lastShownPos_ = -1000000;
};
