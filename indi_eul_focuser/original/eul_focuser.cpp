/*
 * INDI closed-loop encoder focuser for Raspberry Pi 5 -- see eul_focuser.h
 */
#include "eul_focuser.h"

#include <indilogger.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <strings.h>
#include <time.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <memory>

static std::unique_ptr<EulFocuser> focuser(new EulFocuser());

namespace
{
constexpr int kOverrunUm = 100;     // how far past [min,max] we tolerate before aborting outward motion
constexpr int kDirSetupNs = 20000;  // DIR must be stable this long before the first STEP edge
constexpr int kEnableSettleMs = 20; // driver wake-up time after ENABLE
constexpr unsigned IDX_STEP = 0, IDX_DIR = 1, IDX_ENABLE = 2; // order of lines in the GPIO request
}

// =====================================================================================
//  Construction / properties
// =====================================================================================
EulFocuser::EulFocuser()
{
    setVersion(1, 0);
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    setSupportedConnections(CONNECTION_NONE); // no serial/TCP plugin: we manage UDP + GPIO ourselves
}

EulFocuser::~EulFocuser()
{
    stopMotion();
    stopUdp();
}

const char *EulFocuser::getDefaultName()
{
    return "Eul Focuser";
}

bool EulFocuser::initProperties()
{
    INDI::Focuser::initProperties();
    setDefaultPollingPeriod(100);

    const char *dev = getDeviceName();

    UdpNP[U_PORT].fill("UDP_PORT", "UDP port", "%.0f", 1024, 65535, 1, 2345);
    UdpNP[U_STALE_MOVING].fill("STALE_MOVING_MS", "Data timeout moving (ms)", "%.0f", 100, 5000, 50, 500);
    UdpNP[U_STALE_IDLE].fill("STALE_IDLE_MS", "Data timeout idle (ms)", "%.0f", 1500, 60000, 100, 3000);
    UdpNP.fill(dev, "UDP_SETTINGS", "Encoder UDP", "Encoder", IP_RW, 0, IPS_IDLE);

    GpioNP[G_CHIP].fill("GPIOCHIP", "gpiochip #", "%.0f", 0, 15, 1, 4);
    GpioNP[G_STEP].fill("STEP", "STEP line", "%.0f", 0, 60, 1, 23);
    GpioNP[G_DIR].fill("DIR", "DIR line", "%.0f", 0, 60, 1, 24);
    GpioNP[G_ENABLE].fill("ENABLE", "ENABLE line", "%.0f", 0, 60, 1, 25);
    GpioNP.fill(dev, "GPIO_LINES", "GPIO (reconnect to apply)", "Motor", IP_RW, 0, IPS_IDLE);

    MotorNP[M_USTEPS_MM].fill("USTEPS_PER_MM", "usteps / mm", "%.1f", 100, 100000, 10, 1900);
    MotorNP[M_USTEPS_STEP].fill("USTEPS_PER_STEP", "usteps / full step (info)", "%.0f", 1, 256, 1, 32);
    MotorNP[M_DELAY_FAST].fill("DELAY_STEP_US", "Step period, fast (us)", "%.0f", 20, 10000, 5, 125);
    MotorNP[M_DELAY_FINE].fill("DELAY_FINE_US", "Step period, fine zone (us)", "%.0f", 20, 20000, 5, 400);
    MotorNP[M_DELAY_START].fill("DELAY_START_US", "Step period, ramp start (us)", "%.0f", 20, 20000, 5, 1000);
    MotorNP[M_RAMP_STEPS].fill("RAMP_STEPS", "Ramp length (usteps)", "%.0f", 10, 20000, 10, 400);
    MotorNP[M_FINE_ZONE].fill("FINE_ZONE_UM", "Fine zone (um from target)", "%.0f", 0, 2000, 10, 100);
    MotorNP[M_PULSE].fill("PULSE_US", "STEP pulse width (us)", "%.0f", 1, 100, 1, 5);
    MotorNP[M_HOLD_MS].fill("HOLD_MS", "Hold time after move (ms)", "%.0f", 0, 600000, 500, 5000);
    MotorNP.fill(dev, "MOTOR_PARAMS", "Motor", "Motor", IP_RW, 0, IPS_IDLE);

    LoopNP[L_TOL].fill("TOLERANCE_UM", "Tolerance (um)", "%.0f", 1, 500, 1, 1);
    LoopNP[L_PASSES].fill("MAX_PASSES", "Max correction passes", "%.0f", 1, 20, 1, 5);
    LoopNP[L_BACK_IN].fill("BACKLASH_IN_UM", "Backlash, moving in (um)", "%.0f", 0, 2000, 10, 500);
    LoopNP[L_BACK_OUT].fill("BACKLASH_OUT_UM", "Backlash, moving out (um)", "%.0f", 0, 2000, 10, 500);
    LoopNP[L_STALL_MARGIN].fill("STALL_MARGIN_UM", "Stall margin (um)", "%.0f", 50, 5000, 10, 300);
    LoopNP[L_UM_PER_COUNT].fill("ENCODER_UM_PER_COUNT", "Encoder um / count", "%.0f", 1, 1000, 1, 1);
    LoopNP[L_MIN_POS].fill("MIN_POSITION_UM", "Minimum position (um)", "%.0f", 0, 100000, 10, 500);
    LoopNP.fill(dev, "LOOP_PARAMS", "Closed loop", "Motor", IP_RW, 0, IPS_IDLE);

    PosStateNP[P_LAST].fill("LAST_POSITION_UM", "Last position (um)", "%.0f", -1e7, 1e8, 1, 0);
    PosStateNP[P_OFFSET].fill("ENCODER_OFFSET_UM", "Encoder offset (um)", "%.0f", -1e8, 1e8, 1, 0);
    PosStateNP.fill(dev, "POSITION_STATE", "Saved position state", "Encoder", IP_RW, 0, IPS_IDLE);

    StatusNP[S_RAW].fill("ENCODER_RAW", "Encoder raw (counts)", "%.0f", -1e9, 1e9, 1, 0);
    StatusNP[S_POS].fill("POSITION_UM", "Position (um)", "%.0f", -1e8, 1e8, 1, 0);
    StatusNP[S_AGE].fill("UDP_AGE_MS", "Last data age (ms)", "%.0f", 0, 1e9, 1, 0);
    StatusNP.fill(dev, "ENCODER_STATUS", "Encoder status", "Encoder", IP_RO, 0, IPS_IDLE);

    EnablePolSP[0].fill("ENABLE_ACTIVE_LOW", "Active low", ISS_ON);
    EnablePolSP[1].fill("ENABLE_ACTIVE_HIGH", "Active high", ISS_OFF);
    EnablePolSP.fill(dev, "ENABLE_POLARITY", "ENABLE polarity", "Motor", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    DirPolSP[0].fill("DIR_HIGH_INCREASES", "DIR high = position increases", ISS_ON);
    DirPolSP[1].fill("DIR_LOW_INCREASES", "DIR low = position increases", ISS_OFF);
    DirPolSP.fill(dev, "DIR_POLARITY", "DIR polarity", "Motor", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    RampSP[0].fill("RAMP_OFF", "Off", ISS_ON);
    RampSP[1].fill("RAMP_ON", "On", ISS_OFF);
    RampSP.fill(dev, "RAMP", "Acceleration ramp", "Motor", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    RestoreSP[0].fill("RESTORE_OFF", "Use saved offset", ISS_ON);
    RestoreSP[1].fill("RESTORE_ON", "Restore last position", ISS_OFF);
    RestoreSP.fill(dev, "RESTORE_MODE", "Position at connect", "Encoder", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Travel range 500 .. 6000 um
    setMaxPos(6000);
    setAbsLimits(500, 6000);
    setAbsValue(500);

    return true;
}

void EulFocuser::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);

    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    defineProperty(UdpNP);
    defineProperty(GpioNP);
    defineProperty(MotorNP);
    defineProperty(LoopNP);
    defineProperty(PosStateNP);
    defineProperty(EnablePolSP);
    defineProperty(DirPolSP);
    defineProperty(RampSP);
    defineProperty(RestoreSP);

    if (!configLoaded_)
    {
        configLoaded_ = true;
        loadConfig(true, UdpNP.getName());
        loadConfig(true, GpioNP.getName());
        loadConfig(true, MotorNP.getName());
        loadConfig(true, LoopNP.getName());
        loadConfig(true, PosStateNP.getName());
        loadConfig(true, EnablePolSP.getName());
        loadConfig(true, DirPolSP.getName());
        loadConfig(true, RampSP.getName());
        loadConfig(true, RestoreSP.getName());
        refreshCfg();
        setAbsLimits(LoopNP[L_MIN_POS].getValue(), getMaxPos());
    }
}

bool EulFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
        defineProperty(StatusNP);
    else
        deleteProperty(StatusNP.getName());
    return true;
}

bool EulFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);
    UdpNP.save(fp);
    GpioNP.save(fp);
    MotorNP.save(fp);
    LoopNP.save(fp);
    PosStateNP.save(fp);
    EnablePolSP.save(fp);
    DirPolSP.save(fp);
    RampSP.save(fp);
    RestoreSP.save(fp);
    return true;
}

// Copy property values into the plain struct used by the motion thread.
void EulFocuser::refreshCfg()
{
    Cfg c;
    c.ustepsPerMm = MotorNP[M_USTEPS_MM].getValue();
    c.delayFast = MotorNP[M_DELAY_FAST].getValue();
    c.delayFine = std::max(c.delayFast, MotorNP[M_DELAY_FINE].getValue());
    c.delayStart = std::max(c.delayFast, MotorNP[M_DELAY_START].getValue());
    c.rampSteps = (RampSP.findOnSwitchIndex() == 1) ? MotorNP[M_RAMP_STEPS].getValue() : 0;
    c.fineZoneUm = MotorNP[M_FINE_ZONE].getValue();
    c.pulseUs = MotorNP[M_PULSE].getValue();
    c.tolUm = LoopNP[L_TOL].getValue();
    c.maxPasses = static_cast<int>(LoopNP[L_PASSES].getValue());
    c.backIn = LoopNP[L_BACK_IN].getValue();
    c.backOut = LoopNP[L_BACK_OUT].getValue();
    c.stallMarginUm = LoopNP[L_STALL_MARGIN].getValue();
    c.minPos = static_cast<int>(LoopNP[L_MIN_POS].getValue());
    c.maxPos = static_cast<int>(getMaxPos());
    c.staleMovingMs = static_cast<int>(UdpNP[U_STALE_MOVING].getValue());
    c.enableActiveLow = (EnablePolSP.findOnSwitchIndex() != 1);
    c.dirHighIncreases = (DirPolSP.findOnSwitchIndex() != 1);
    {
        std::lock_guard<std::mutex> lk(cfgMutex_);
        cfg_ = c;
    }
    scaleUm_ = static_cast<int>(LoopNP[L_UM_PER_COUNT].getValue());
}

// =====================================================================================
//  Property handlers
// =====================================================================================
bool EulFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        auto handle = [&](INDI::PropertyNumber & p) -> bool
        {
            if (!p.isNameMatch(name))
                return false;
            bool ok = p.update(values, names, n);
            p.setState(ok ? IPS_OK : IPS_ALERT);
            p.apply();
            return true;
        };

        if (handle(UdpNP))
        {
            refreshCfg();
            if (isConnected())
            {
                stopUdp();
                std::string err;
                if (!startUdp(err))
                {
                    LOGF_ERROR("Cannot (re)open UDP port: %s", err.c_str());
                    UdpNP.setState(IPS_ALERT);
                    UdpNP.apply();
                }
                else
                    LOGF_INFO("Listening on UDP port %.0f", UdpNP[U_PORT].getValue());
            }
            return true;
        }
        if (handle(GpioNP))
        {
            if (isConnected())
                LOG_WARN("GPIO line changes take effect after disconnect/reconnect.");
            return true;
        }
        if (handle(MotorNP))
        {
            refreshCfg();
            return true;
        }
        if (handle(LoopNP))
        {
            refreshCfg();
            setAbsLimits(LoopNP[L_MIN_POS].getValue(), getMaxPos());
            applyAbs();
            return true;
        }
        if (handle(PosStateNP))
        {
            offsetUm_ = static_cast<int>(PosStateNP[P_OFFSET].getValue());
            return true;
        }
    }
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool EulFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        auto handle = [&](INDI::PropertySwitch & p) -> bool
        {
            if (!p.isNameMatch(name))
                return false;
            bool ok = p.update(states, names, n);
            p.setState(ok ? IPS_OK : IPS_ALERT);
            p.apply();
            return true;
        };

        if (handle(EnablePolSP))
        {
            refreshCfg();
            if (gpio_.isOpen() && !moving_)
                setEnable(motorEnabled_, EnablePolSP.findOnSwitchIndex() != 1);
            return true;
        }
        if (handle(DirPolSP) || handle(RampSP) || handle(RestoreSP))
        {
            refreshCfg();
            return true;
        }
    }
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

// =====================================================================================
//  Connect / disconnect
// =====================================================================================
bool EulFocuser::Connect()
{
    refreshCfg();

    Cfg c;
    {
        std::lock_guard<std::mutex> lk(cfgMutex_);
        c = cfg_;
    }

    // GPIO: STEP idle low, DIR low, ENABLE at its "disabled" level.
    unsigned offsets[3];
    offsets[IDX_STEP] = static_cast<unsigned>(GpioNP[G_STEP].getValue());
    offsets[IDX_DIR] = static_cast<unsigned>(GpioNP[G_DIR].getValue());
    offsets[IDX_ENABLE] = static_cast<unsigned>(GpioNP[G_ENABLE].getValue());
    int initial[3] = {0, 0, 0};
    initial[IDX_ENABLE] = c.enableActiveLow ? 1 : 0;

    std::string chip = "/dev/gpiochip" + std::to_string(static_cast<int>(GpioNP[G_CHIP].getValue()));
    std::string err;
    if (!gpio_.request(chip, offsets, initial, 3, "indi_eul_focuser", err))
    {
        LOGF_ERROR("GPIO setup failed: %s", err.c_str());
        return false;
    }
    motorEnabled_ = false;

    // Encoder bookkeeping
    encValid_ = false;
    lastRxNs_ = 0;
    offsetUm_ = static_cast<int>(PosStateNP[P_OFFSET].getValue());
    restorePending_ = (RestoreSP.findOnSwitchIndex() == 1);
    staleReported_ = false;
    rtReported_ = false;
    tick_ = 0;

    if (!startUdp(err))
    {
        LOGF_ERROR("UDP setup failed: %s", err.c_str());
        gpio_.release();
        return false;
    }
    LOGF_INFO("GPIO %s STEP=%u DIR=%u ENABLE=%u ready; listening on UDP port %.0f.", chip.c_str(),
              offsets[IDX_STEP], offsets[IDX_DIR], offsets[IDX_ENABLE], UdpNP[U_PORT].getValue());
    if (restorePending_)
        LOG_INFO("Waiting for first encoder reading to restore the last saved position.");

    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool EulFocuser::Disconnect()
{
    stopMotion();
    if (encValid_)
        persistPosition();
    if (gpio_.isOpen())
        setEnable(false, EnablePolSP.findOnSwitchIndex() != 1);
    stopUdp();
    gpio_.release();
    return true;
}

// =====================================================================================
//  UDP receiver
// =====================================================================================
bool EulFocuser::startUdp(std::string &err)
{
    int port = static_cast<int>(UdpNP[U_PORT].getValue());
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) { err = strerror(errno); return false; }

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        err = "bind port " + std::to_string(port) + ": " + strerror(errno);
        close(fd);
        return false;
    }
    udpFd_ = fd;
    udpRun_ = true;
    udpThread_ = std::thread(&EulFocuser::udpLoop, this);
    return true;
}

void EulFocuser::stopUdp()
{
    udpRun_ = false;
    if (udpThread_.joinable())
        udpThread_.join();
    if (udpFd_ >= 0)
    {
        close(udpFd_);
        udpFd_ = -1;
    }
}

void EulFocuser::udpLoop()
{
    char buf[2048];
    while (udpRun_)
    {
        pollfd pfd;
        pfd.fd = udpFd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int r = poll(&pfd, 1, 200);
        if (r <= 0)
            continue;
        ssize_t len = recv(udpFd_, buf, sizeof(buf) - 1, 0);
        if (len <= 0)
            continue;
        buf[len] = '\0';
        parseDatagram(buf, static_cast<size_t>(len));
    }
}

// A datagram may hold one or several lines. "ypr..." lines (IMU) are ignored; an encoder
// line is a plain (optionally signed) integer in encoder counts. Last valid line wins.
void EulFocuser::parseDatagram(const char *buf, size_t len)
{
    std::string s(buf, len);
    char *save = nullptr;
    for (char *line = strtok_r(&s[0], "\r\n", &save); line != nullptr; line = strtok_r(nullptr, "\r\n", &save))
    {
        const char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            continue;
        if (strncasecmp(p, "ypr", 3) == 0)
            continue;

        char *end = nullptr;
        errno = 0;
        long v = strtol(p, &end, 10);
        if (end == p || errno != 0)
            continue;
        while (*end == ' ' || *end == '\t')
            end++;
        if (*end != '\0')
            continue; // not "figures only": ignore

        encRaw_ = static_cast<int>(v);
        lastRxNs_ = nowNs();
        encValid_ = true;
    }
}

int EulFocuser::positionUm() const
{
    return encRaw_.load() * scaleUm_.load() + offsetUm_.load();
}

int64_t EulFocuser::dataAgeMs() const
{
    if (!encValid_)
        return INT64_C(1000000000);
    return (nowNs() - lastRxNs_.load()) / 1000000;
}

// =====================================================================================
//  Motor control
// =====================================================================================
void EulFocuser::setEnable(bool on, bool activeLow)
{
    int level = on ? (activeLow ? 0 : 1) : (activeLow ? 1 : 0);
    gpio_.set(IDX_ENABLE, level);
    motorEnabled_ = on;
}

int64_t EulFocuser::nowNs()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Sleep for the bulk of the interval, spin for the last ~100 us.
void EulFocuser::waitUntil(int64_t target)
{
    for (;;)
    {
        int64_t rem = target - nowNs();
        if (rem <= 0)
            return;
        if (rem > 200000)
        {
            int64_t t = target - 100000;
            timespec ts;
            ts.tv_sec = t / 1000000000LL;
            ts.tv_nsec = t % 1000000000LL;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
        }
    }
}

void EulFocuser::stopMotion()
{
    abortReq_ = true;
    if (moveThread_.joinable())
        moveThread_.join();
    abortReq_ = false;
    moving_ = false;
}

IPState EulFocuser::startMove(int target)
{
    if (!gpio_.isOpen())
    {
        LOG_ERROR("GPIO not available.");
        return IPS_ALERT;
    }
    if (restorePending_ || !encValid_)
    {
        LOG_ERROR("No encoder reading yet; cannot move.");
        return IPS_ALERT;
    }
    if (dataAgeMs() > UdpNP[U_STALE_IDLE].getValue())
    {
        LOGF_ERROR("Encoder data is stale (%lld ms old); cannot move.", static_cast<long long>(dataAgeMs()));
        return IPS_ALERT;
    }

    stopMotion();
    moveDone_ = false;
    refreshCfg();
    Cfg c;
    {
        std::lock_guard<std::mutex> lk(cfgMutex_);
        c = cfg_;
    }

    target = std::max(c.minPos, std::min(c.maxPos, target));
    if (std::abs(target - positionUm()) <= c.tolUm)
        return IPS_OK;

    moveTarget_ = target;
    moving_ = true;
    LOGF_INFO("Moving from %d to %d um.", positionUm(), target);
    moveThread_ = std::thread(&EulFocuser::motionThread, this, target, c);
    return IPS_BUSY;
}

// Closed-loop move: step toward the target, re-evaluating the encoder before every pulse.
void EulFocuser::motionThread(int target, Cfg c)
{
    // Real-time scheduling for steady step timing; works if rtprio is allowed for the user.
    sched_param sp;
    sp.sched_priority = c.rtPrio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        rtFailed_ = true;

    MoveResult res = MoveResult::Ok;
    const double k = c.ustepsPerMm / 1000.0; // usteps per um

    if (!motorEnabled_)
    {
        setEnable(true, c.enableActiveLow);
        waitUntil(nowNs() + kEnableSettleMs * 1000000LL);
    }

    int dir = 0;                      // direction of the current run (+1 = position increasing)
    int physDir = lastMoveDir_.load(); // last direction the motor really turned (for backlash)
    int pass = 0;
    uint64_t stepsDone = 0, sinceChange = 0;
    int lastRaw = encRaw_.load();
    bool changed = false;
    double slackUm = 0;
    int64_t next = nowNs();

    for (;;)
    {
        if (abortReq_) { res = MoveResult::Aborted; break; }

        int64_t now = nowNs();
        if ((now - lastRxNs_.load()) / 1000000 > c.staleMovingMs) { res = MoveResult::StaleData; break; }

        int pos = positionUm();
        int err = target - pos;
        int aerr = std::abs(err);
        if (aerr <= c.tolUm) { res = MoveResult::Ok; break; }

        int nd = (err > 0) ? 1 : -1;
        if (nd != dir)
        {
            if (dir != 0 && ++pass > c.maxPasses) { res = MoveResult::MaxPasses; break; }
            dir = nd;
            int level = ((dir > 0) == c.dirHighIncreases) ? 1 : 0;
            if (!gpio_.set(IDX_DIR, level)) { res = MoveResult::GpioError; break; }
            waitUntil(nowNs() + kDirSetupNs);

            // Direction reversal => mechanical slack to take up before the encoder responds.
            slackUm = (physDir != 0 && dir != physDir) ? (dir > 0 ? c.backOut : c.backIn) : 0;
            physDir = dir;
            stepsDone = 0;
            sinceChange = 0;
            changed = false;
            lastRaw = encRaw_.load();
            next = nowNs();
        }

        if ((dir > 0 && pos > c.maxPos + kOverrunUm) || (dir < 0 && pos < c.minPos - kOverrunUm))
        {
            res = MoveResult::Limit;
            break;
        }

        // Step period: fast, optional trapezoid ramp, slower inside the fine zone.
        double d = c.delayFast;
        if (c.rampSteps > 0 && c.delayStart > c.delayFast)
        {
            double remSteps = aerr * k;
            double span = c.delayStart - c.delayFast;
            double a = c.delayStart - span * std::min<double>(stepsDone, c.rampSteps) / c.rampSteps;
            double b = c.delayStart - span * std::min<double>(remSteps, c.rampSteps) / c.rampSteps;
            d = std::max(a, b);
        }
        if (aerr < c.fineZoneUm)
            d = std::max(d, c.delayFine);

        // One step pulse
        if (!gpio_.set(IDX_STEP, 1)) { res = MoveResult::GpioError; break; }
        waitUntil(nowNs() + static_cast<int64_t>(c.pulseUs * 1000.0));
        gpio_.set(IDX_STEP, 0);
        stepsDone++;

        // Stall detection: steps issued without any encoder change
        int raw = encRaw_.load();
        if (raw != lastRaw) { lastRaw = raw; sinceChange = 0; changed = true; }
        else sinceChange++;
        double limit = (c.stallMarginUm + (changed ? 0.0 : slackUm)) * k;
        if (sinceChange > limit) { res = MoveResult::Stall; break; }

        next += static_cast<int64_t>(d * 1000.0);
        int64_t t = nowNs();
        if (next < t)
            next = t; // we are late; do not burst to catch up
        waitUntil(next);
    }

    gpio_.set(IDX_STEP, 0);
    lastMoveDir_ = physDir;
    moveResult_ = static_cast<int>(res);
    moveDone_ = true;
}

// Called from the main thread when the motion thread has finished.
void EulFocuser::finishMove()
{
    if (moveThread_.joinable())
        moveThread_.join();
    moving_ = false;
    lastMoveEndNs_ = nowNs();

    MoveResult res = static_cast<MoveResult>(moveResult_.load());
    int pos = positionUm();
    IPState st = IPS_OK;
    switch (res)
    {
        case MoveResult::Ok:
            LOGF_INFO("Reached %d um (target %d, error %d um).", pos, moveTarget_.load(), pos - moveTarget_.load());
            break;
        case MoveResult::Aborted:
            LOGF_WARN("Move aborted at %d um.", pos);
            st = IPS_IDLE;
            break;
        case MoveResult::Stall:
            LOGF_ERROR("Stall: motor stepped but the encoder did not move. Stopped at %d um.", pos);
            st = IPS_ALERT;
            break;
        case MoveResult::StaleData:
            LOGF_ERROR("Encoder data lost during move. Stopped at %d um.", pos);
            st = IPS_ALERT;
            break;
        case MoveResult::MaxPasses:
            LOGF_ERROR("Could not settle within tolerance after %.0f passes. Stopped at %d um.",
                       LoopNP[L_PASSES].getValue(), pos);
            st = IPS_ALERT;
            break;
        case MoveResult::Limit:
            LOGF_ERROR("Travel limit exceeded. Stopped at %d um.", pos);
            st = IPS_ALERT;
            break;
        case MoveResult::GpioError:
            LOG_ERROR("GPIO write failed during move.");
            st = IPS_ALERT;
            break;
    }
    setAbsValue(pos);
    setAbsState(st);
    setRelState(st);
    applyAbs();
    applyRel();
    persistPosition();
}

void EulFocuser::persistPosition()
{
    PosStateNP[P_LAST].setValue(positionUm());
    PosStateNP[P_OFFSET].setValue(offsetUm_.load());
    PosStateNP.setState(IPS_OK);
    PosStateNP.apply();
    saveConfig(true, PosStateNP.getName());
}

// =====================================================================================
//  Focuser interface
// =====================================================================================
IPState EulFocuser::MoveAbsFocuser(uint32_t target)
{
    return startMove(static_cast<int>(target));
}

IPState EulFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // INDI convention: outward = position increases.
    int delta = (dir == FOCUS_OUTWARD) ? static_cast<int>(ticks) : -static_cast<int>(ticks);
    int target = positionUm() + delta;
    return startMove(target);
}

bool EulFocuser::AbortFocuser()
{
    abortReq_ = true; // the motion thread stops at its next step; TimerHit reports it
    return true;
}

bool EulFocuser::SyncFocuser(uint32_t ticks)
{
    if (!encValid_)
    {
        LOG_ERROR("Cannot sync: no encoder reading.");
        return false;
    }
    if (moving_)
    {
        LOG_ERROR("Cannot sync while moving.");
        return false;
    }
    offsetUm_ = static_cast<int>(ticks) - encRaw_.load() * scaleUm_.load();
    setAbsValue(ticks);
    applyAbs();
    persistPosition();
    LOGF_INFO("Synced: position is now %u um (offset %d um).", ticks, offsetUm_.load());
    return true;
}

bool EulFocuser::SetFocuserMaxPosition(uint32_t ticks)
{
    setAbsLimits(LoopNP[L_MIN_POS].getValue(), ticks);
    refreshCfg();
    return true;
}

// =====================================================================================
//  Periodic housekeeping (100 ms)
// =====================================================================================
void EulFocuser::TimerHit()
{
    if (!isConnected())
        return;

    tick_++;

    // One-time position restore once the first reading is available.
    if (restorePending_ && encValid_)
    {
        offsetUm_ = static_cast<int>(PosStateNP[P_LAST].getValue()) - encRaw_.load() * scaleUm_.load();
        restorePending_ = false;
        LOGF_INFO("Restored position: %d um (offset %d um).", positionUm(), offsetUm_.load());
        persistPosition();
    }

    if (rtFailed_ && !rtReported_)
    {
        rtReported_ = true;
        LOG_WARN("Could not get SCHED_FIFO for the step thread (set 'rtprio' in limits.conf). Timing may jitter.");
    }

    if (moveDone_.exchange(false))
        finishMove();

    int pos = positionUm();
    if (encValid_ && pos != lastShownPos_)
    {
        lastShownPos_ = pos;
        setAbsValue(pos);
        applyAbs();
    }

    // Data freshness while idle
    int64_t age = dataAgeMs();
    bool stale = !moving_ && (age > UdpNP[U_STALE_IDLE].getValue());
    if (stale && !staleReported_)
    {
        staleReported_ = true;
        LOGF_WARN("No encoder data for %lld ms.", static_cast<long long>(age));
        StatusNP.setState(IPS_ALERT);
        StatusNP.apply();
    }
    else if (!stale && staleReported_ && encValid_)
    {
        staleReported_ = false;
        LOG_INFO("Encoder data is back.");
        StatusNP.setState(IPS_OK);
        StatusNP.apply();
    }

    // Release the motor after the hold time.
    if (!moving_ && motorEnabled_ && gpio_.isOpen())
    {
        double holdMs = MotorNP[M_HOLD_MS].getValue();
        if ((nowNs() - lastMoveEndNs_) / 1000000 >= holdMs)
        {
            setEnable(false, EnablePolSP.findOnSwitchIndex() != 1);
            LOG_DEBUG("Motor disabled after hold time.");
        }
    }

    if (tick_ % 5 == 0 && encValid_)
    {
        StatusNP[S_RAW].setValue(encRaw_.load());
        StatusNP[S_POS].setValue(pos);
        StatusNP[S_AGE].setValue(static_cast<double>(age));
        if (!staleReported_)
            StatusNP.setState(IPS_OK);
        StatusNP.apply();
    }

    SetTimer(getCurrentPollingPeriod());
}

// =====================================================================================
//  Access to FocuserInterface properties (legacy structs vs. INDI::PropertyNumber)
// =====================================================================================
#if EF_FOCUS_WRAPPED
void EulFocuser::setAbsValue(double v) { FocusAbsPosNP[0].setValue(v); }
void EulFocuser::setAbsState(IPState s) { FocusAbsPosNP.setState(s); }
void EulFocuser::applyAbs() { FocusAbsPosNP.apply(); }
void EulFocuser::setRelState(IPState s) { FocusRelPosNP.setState(s); }
void EulFocuser::applyRel() { FocusRelPosNP.apply(); }
void EulFocuser::setAbsLimits(double mn, double mx)
{
    FocusAbsPosNP[0].setMin(mn);
    FocusAbsPosNP[0].setMax(mx);
    FocusAbsPosNP.updateMinMax();
}
double EulFocuser::getMaxPos() { return FocusMaxPosNP[0].getValue(); }
void EulFocuser::setMaxPos(double v) { FocusMaxPosNP[0].setValue(v); }
#else
void EulFocuser::setAbsValue(double v) { FocusAbsPosN[0].value = v; }
void EulFocuser::setAbsState(IPState s) { FocusAbsPosNP.s = s; }
void EulFocuser::applyAbs() { IDSetNumber(&FocusAbsPosNP, nullptr); }
void EulFocuser::setRelState(IPState s) { FocusRelPosNP.s = s; }
void EulFocuser::applyRel() { IDSetNumber(&FocusRelPosNP, nullptr); }
void EulFocuser::setAbsLimits(double mn, double mx)
{
    FocusAbsPosN[0].min = mn;
    FocusAbsPosN[0].max = mx;
    IUUpdateMinMax(&FocusAbsPosNP);
}
double EulFocuser::getMaxPos() { return FocusMaxPosN[0].value; }
void EulFocuser::setMaxPos(double v) { FocusMaxPosN[0].value = v; }
#endif
