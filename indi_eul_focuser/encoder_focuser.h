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
#include <cmath>

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

class EncoderFocuser : public INDI::Focuser
{
    public:
        EncoderFocuser();
        ~EncoderFocuser() override;

        const char *getDefaultName() override;

        bool initProperties() override;
        void ISGetProperties(const char *dev) override;
        bool updateProperties() override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

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
            double tolUm = 20, backIn = 200, backOut = 200, stallMarginUm = 300;
            int maxPasses = 5;
            int minPos = 500, maxPos = 6000;
            int staleMovingMs = 500;
            bool enableActiveLow = true, dirHighIncreases = true;
            int rtPrio = 50;
        };

        enum class MoveResult : int { Ok = 0, Aborted, Stall, StaleData, MaxPasses, Limit, GpioError };

        // ---- IMU safety (quaternion or "ypr" lines on the same UDP stream) ----------------
        void updateSafety();
        static bool parseQuaternion(const char *p, double &w, double &x, double &y, double &z);
        void setAngles(double yaw, double pitch, double roll);

        // ---- ESP32 command port (telnet-style "s <value>") ---------------------------------
        bool espCommandSet(int um, std::string &err);   // send "s <um>", wait for "OK"
        bool espPush(int um, std::string &err);          // espCommandSet + wait for a fresh reading

        // ---- helpers ---------------------------------------------------------------------
        void refreshCfg();
        struct UdpIn
        {
            int fd = -1;
            std::thread th;
            std::atomic<bool> run{false};
        };
        bool startUdp(std::string &err);   // encoder port, plus the IMU port when it is different
        void stopUdp();
        bool openUdp(UdpIn &u, int port, bool isMain, std::string &err);
        void closeUdp(UdpIn &u);
        void udpLoop(UdpIn *u, bool isMain);
        bool restartImuUdp();
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
        INDI::PropertySwitch RestoreSP  {3};
        INDI::PropertyNumber ImuNP      {9};   // centre/tolerance per axis, hold time, data timeout
        INDI::PropertySwitch ImuCheckSP {3};   // which axes take part in the safety check
        INDI::PropertySwitch ImuCaptureSP{1};  // button: centre := current angles
        INDI::PropertyNumber ImuAnglesNP{3};   // read only
        INDI::PropertyLight  SafetyLP   {1};   // SCOPE_SAFETY / SAFE, snooped by the roof driver
        INDI::PropertyText   EspTP      {1};   // ESP32 host (empty = sender of the UDP datagrams)
        INDI::PropertyNumber EspNP      {2};   // ESP32 command port, timeout
        INDI::PropertyNumber EspSetNP   {1};   // action: set the ESP32 encoder to this value (um)
        INDI::PropertySwitch SyncPushSP {2};   // Sync: change the offset (default) / set the ESP32 encoder

        enum { U_PORT, U_STALE_MOVING, U_STALE_IDLE };
        enum { G_CHIP, G_STEP, G_DIR, G_ENABLE };
        enum { M_USTEPS_MM, M_USTEPS_STEP, M_DELAY_FAST, M_DELAY_FINE, M_DELAY_START, M_RAMP_STEPS,
               M_FINE_ZONE, M_PULSE, M_HOLD_MS };
        enum { L_TOL, L_PASSES, L_BACK_IN, L_BACK_OUT, L_STALL_MARGIN, L_UM_PER_COUNT, L_MIN_POS };
        enum { P_LAST, P_OFFSET };
        enum { S_RAW, S_POS, S_AGE };
        enum { E_PORT, E_TIMEOUT };
        enum { I_YAW_C, I_PITCH_C, I_ROLL_C, I_YAW_T, I_PITCH_T, I_ROLL_T, I_HOLD_MS, I_TIMEOUT_MS, I_UDP_PORT };

        // ---- runtime state ---------------------------------------------------------------
        Cfg cfg_;
        std::mutex cfgMutex_;

        GpioOutputs gpio_;
        std::atomic<bool> motorEnabled_{false};

        // UDP
        UdpIn udpMain_;
        UdpIn udpImu_;
        std::atomic<int>  encRaw_{0};
        std::atomic<bool> encValid_{false};
        std::atomic<int64_t> lastRxNs_{0};
        std::atomic<int>  scaleUm_{1};
        std::atomic<int>  offsetUm_{0};

        // IMU
        std::atomic<double> imuYaw_{0}, imuPitch_{0}, imuRoll_{0};
        std::atomic<bool>   imuValid_{false};
        std::atomic<uint32_t> lastSenderIp_{0};   // IPv4 (network order) of the last UDP datagram
        int64_t restoreRetryNs_ = 0;
        std::atomic<int64_t> lastImuNs_{0};
        bool    imuSafe_ = false;
        int64_t imuSafeSinceNs_ = 0;
        int64_t lastSafetyPubNs_ = 0;
        bool    lastSafetyPub_ = false;

        // motion
        std::thread moveThread_;
        std::atomic<bool> abortReq_{false};
        std::atomic<bool> moving_{false};
        std::atomic<bool> moveDone_{false};
        std::atomic<int>  moveResult_{0};
        std::atomic<int>  lastMoveDir_{0};
        std::atomic<bool> rtFailed_{false};
        std::atomic<int>  moveTarget_{0};
        std::atomic<int>  measuredSlackUm_{-1}; // um of motor travel before the encoder responded after a reversal
        std::atomic<int>  stallTravelUm_{0};    // um of motor travel without encoder change at stall
        std::atomic<int>  stallAllowUm_{0};     // allowance that was in force
        int64_t lastMoveEndNs_ = 0;

        bool configLoaded_ = false;
        bool restorePending_ = false;
        bool staleReported_ = false;
        bool rtReported_ = false;
        int  tick_ = 0;
        int  lastShownPos_ = -1000000;
};
