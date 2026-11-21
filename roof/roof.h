/*
 * INDI Dome driver for a roll-off roof driven through relays.
 *
 *   - OPEN / STOP / CLOSE = timed pulses on three relays of the 16-port HTTP box
 *   - motor power = one relay of the KMTronic 8 box (cut whenever it is not safe to move)
 *   - safety: mount parked (snoops TELESCOPE_PARK of the mount driver) AND scope inside the
 *     IMU safe zone (snoops SCOPE_SAFETY of the encoder-focuser driver)
 *   - no position feedback: state follows travel timers; the end limit switches stop the motor
 *
 * All drivers must run in the same indiserver for snooping to work.
 */
#pragma once

#include <indidome.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include "../common/kmtronic_cmd.h"

class RollOffRoof : public INDI::Dome
{
    public:
        RollOffRoof();
        ~RollOffRoof() override;
        const char *getDefaultName() override;

        bool initProperties() override;
        void ISGetProperties(const char *dev) override;
        bool updateProperties() override;
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        bool ISSnoopDevice(XMLEle *root) override;

    protected:
        bool Connect() override;
        bool Disconnect() override;
        void TimerHit() override;
        bool saveConfigItems(FILE *fp) override;

        bool Abort() override;
        IPState Park() override;
        IPState UnPark() override;
        IPState ControlShutter(ShutterOperation operation) override;

    private:
        // ---- configuration snapshot handed to the worker with every job -------------------
        struct Cfg
        {
            std::string relayHost, powerHost, powerUser, powerPass;
            int relayPort = 80, relayAddr = 30, relayTimeout = 2000;
            int relayStop = 1, relayOpen = 2, relayClose = 3, pulseMs = 500;
            bool pulseRelayOn = true;      // true: relay ON during the pulse (normally open wiring)
            int powerPort = 80, powerTimeout = 2000, motorRelay = 4;
        };
        enum JobType { J_PULSE, J_POWER, J_STATUS };
        struct Job
        {
            JobType type;
            int arg;
            Cfg cfg;
        };

        // worker
        void workerLoop();
        void runJob(const Job &j);
        void post(JobType t, int arg, bool front = false);
        void clearMotionJobs();
        Cfg makeCfg();
        static bool httpGet(const std::string &host, int port, const std::string &path, int timeoutMs,
                            const std::string &user, const std::string &pass, std::string &body, std::string &err);
        static bool readMotorRelay(const Cfg &c, int &state, std::string &err);
        void setWorkerError(const std::string &e);

        // logic
        void evaluateSafety();
        bool moveAllowed(std::string &reason) const;
        IPState startMotion(bool open, bool viaPark);
        void finishMotion();
        void emergencyStop(const char *why);
        void applyMark(int idx, bool save);
        void resnoop();
        bool policy(int idx) const;
        static int64_t nowNs();

        // ---- properties ---------------------------------------------------------------------
        INDI::PropertyText   RelayHostTP {1};
        INDI::PropertyNumber RelayNP     {7};
        INDI::PropertySwitch PulsePolSP  {2};
        INDI::PropertyText   PowerHostTP {3};
        INDI::PropertyNumber PowerNP     {3};
        INDI::PropertyText   SourcesTP   {2};
        INDI::PropertySwitch PolicySP    {4};
        INDI::PropertySwitch PowerStyleSP{3};
        INDI::PropertyNumber TravelNP    {2};
        INDI::PropertySwitch MarkSP      {3};
        INDI::PropertyLight  StatusLP    {4};

        enum { R_PORT, R_ADDR, R_TIMEOUT, R_STOP, R_OPEN, R_CLOSE, R_PULSE };
        enum { H_HOST, H_USER, H_PASS };
        enum { P_PORT, P_TIMEOUT, P_RELAY };
        enum { SRC_MOUNT, SRC_FOCUSER };
        enum { POL_PARKED, POL_IMU, POL_CUT, POL_OVERRIDE };
        enum { T_OPEN, T_CLOSE };
        enum { MARK_OPEN, MARK_CLOSED, MARK_UNKNOWN };
        enum { ST_PARKED, ST_IMU, ST_POWER, ST_ALLOWED };

        // worker state
        std::thread worker_;
        std::mutex qm_;
        std::condition_variable qcv_;
        std::deque<Job> q_;
        bool workerRun_ = false;
        std::atomic<int> pendingPower_{0}, pendingStatus_{0};
        std::atomic<int> kmStyle_{0};       // kmt::Style used by the worker
        std::atomic<bool> styleSwitched_{false};
        std::atomic<int> powerState_{-1};   // -1 unknown, 0 off, 1 on (as last read/commanded)
        std::mutex errM_;
        std::string workerErr_;
        bool workerErrNew_ = false;

        // snooped state
        bool mountKnown_ = false, mountParked_ = false;
        bool imuKnown_ = false, imuSafe_ = false;
        int64_t imuSeenNs_ = 0;

        // roof state machine
        enum Motion { M_IDLE, M_OPENING, M_CLOSING };
        Motion motion_ = M_IDLE;
        bool viaPark_ = false;
        int64_t motionEndNs_ = 0;

        bool allowed_ = false;
        std::string lastReason_;
        bool lastAllowedPub_ = false;
        int64_t lastPowerCmdNs_ = 0, lastStatusReqNs_ = 0;
        bool configLoaded_ = false;
        bool failingLogged_ = false;
        int tickPub_ = 0;
};
