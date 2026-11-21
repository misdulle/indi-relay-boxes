/*
 * INDI closed-loop encoder focuser for Raspberry Pi 5 -- see encoder_focuser.h
 */
#include "encoder_focuser.h"

#include <indilogger.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <poll.h>
#include <netdb.h>
#include <fcntl.h>
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

static std::unique_ptr<EncoderFocuser> focuser(new EncoderFocuser());

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
EncoderFocuser::EncoderFocuser()
{
    setVersion(1, 0);
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    setSupportedConnections(CONNECTION_NONE); // no serial/TCP plugin: we manage UDP + GPIO ourselves
}

EncoderFocuser::~EncoderFocuser()
{
    stopMotion();
    stopUdp();
}

const char *EncoderFocuser::getDefaultName()
{
    return "EulFocuser";
}

bool EncoderFocuser::initProperties()
{
    INDI::Focuser::initProperties();
    setDefaultPollingPeriod(100);

    const char *dev = getDeviceName();

    UdpNP[U_PORT].fill("UDP_PORT", "UDP port", "%.0f", 1024, 65535, 1, 5005);
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

    LoopNP[L_TOL].fill("TOLERANCE_UM", "Tolerance (um)", "%.0f", 1, 500, 1, 20);
    LoopNP[L_PASSES].fill("MAX_PASSES", "Max correction passes", "%.0f", 1, 20, 1, 5);
    LoopNP[L_BACK_IN].fill("BACKLASH_IN_UM", "Backlash, moving in (um)", "%.0f", 0, 2000, 10, 200);
    LoopNP[L_BACK_OUT].fill("BACKLASH_OUT_UM", "Backlash, moving out (um)", "%.0f", 0, 2000, 10, 200);
    LoopNP[L_STALL_MARGIN].fill("STALL_MARGIN_UM", "Stall margin (um)", "%.0f", 50, 5000, 10, 300);
    LoopNP[L_UM_PER_COUNT].fill("ENCODER_UM_PER_COUNT", "Encoder um / count", "%.0f", 1, 1000, 1, 1);
    LoopNP.fill(dev, "LOOP_PARAMS", "Closed loop", "Motor", IP_RW, 0, IPS_IDLE);

    LimitsNP[LIM_MIN].fill("MIN_POSITION_UM", "Minimum position (um)", "%.0f", 0, 100000, 10, 500);
    LimitsNP[LIM_MAX].fill("MAX_POSITION_UM", "Maximum position (um)", "%.0f", 0, 1000000, 10, 6000);
    LimitsNP.fill(dev, "TRAVEL_LIMITS", "Travel limits", "Main Control", IP_RW, 0, IPS_IDLE);

    PosStateNP[P_LAST].fill("LAST_POSITION_UM", "Last position (um)", "%.0f", -1e7, 1e8, 1, 0);
    PosStateNP[P_OFFSET].fill("ENCODER_OFFSET_UM", "Encoder offset (um)", "%.0f", -1e8, 1e8, 1, 0);
    PosStateNP.fill(dev, "POSITION_STATE", "Saved position state", "Encoder", IP_RW, 0, IPS_IDLE);

    StatusNP[S_RAW].fill("ENCODER_RAW", "Encoder raw (counts)", "%.0f", -1e9, 1e9, 1, 0);
    StatusNP[S_POS].fill("POSITION_UM", "Position (um)", "%.0f", -1e8, 1e8, 1, 0);
    StatusNP[S_AGE].fill("UDP_AGE_MS", "Last data age (ms)", "%.0f", 0, 1e9, 1, 0);
    StatusNP.fill(dev, "ENCODER_STATUS", "Encoder status", "Encoder", IP_RO, 0, IPS_IDLE);

    // --- IMU safety: safe when every enabled axis is within centre +/- tolerance -----------
    ImuNP[I_YAW_C].fill("YAW_CENTER", "Yaw centre (deg)", "%.1f", -180, 180, 1, 0);
    ImuNP[I_PITCH_C].fill("PITCH_CENTER", "Pitch centre (deg)", "%.1f", -180, 180, 1, 0);
    ImuNP[I_ROLL_C].fill("ROLL_CENTER", "Roll centre (deg)", "%.1f", -180, 180, 1, 0);
    ImuNP[I_YAW_T].fill("YAW_TOL", "Yaw tolerance +/- (deg)", "%.1f", 0, 180, 0.5, 5);
    ImuNP[I_PITCH_T].fill("PITCH_TOL", "Pitch tolerance +/- (deg)", "%.1f", 0, 180, 0.5, 5);
    ImuNP[I_ROLL_T].fill("ROLL_TOL", "Roll tolerance +/- (deg)", "%.1f", 0, 180, 0.5, 5);
    ImuNP[I_HOLD_MS].fill("SAFE_HOLD_MS", "Must stay in range (ms) before safe", "%.0f", 0, 60000, 100, 2000);
    ImuNP[I_TIMEOUT_MS].fill("DATA_TIMEOUT_MS", "IMU data timeout (ms)", "%.0f", 500, 60000, 100, 3000);
    ImuNP[I_UDP_PORT].fill("IMU_UDP_PORT", "IMU UDP port (0 = encoder port)", "%.0f", 0, 65535, 1, 2345);
    ImuNP.fill(dev, "IMU_LIMITS", "IMU safe zone", "Safety", IP_RW, 0, IPS_IDLE);

    ImuCheckSP[0].fill("CHECK_YAW", "Check yaw", ISS_OFF);
    ImuCheckSP[1].fill("CHECK_PITCH", "Check pitch", ISS_OFF);
    ImuCheckSP[2].fill("CHECK_ROLL", "Check roll", ISS_OFF);
    ImuCheckSP.fill(dev, "IMU_CHECK", "Axes checked", "Safety", IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    ImuCaptureSP[0].fill("SET_CENTER_NOW", "Centre := current angles", ISS_OFF);
    ImuCaptureSP.fill(dev, "IMU_CAPTURE", "Capture", "Safety", IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    ImuAnglesNP[0].fill("YAW", "Yaw (deg)", "%.2f", -360, 360, 0, 0);
    ImuAnglesNP[1].fill("PITCH", "Pitch (deg)", "%.2f", -360, 360, 0, 0);
    ImuAnglesNP[2].fill("ROLL", "Roll (deg)", "%.2f", -360, 360, 0, 0);
    ImuAnglesNP.fill(dev, "IMU_ANGLES", "IMU angles", "Safety", IP_RO, 0, IPS_IDLE);

    SafetyLP[0].fill("SAFE", "Scope in safe position", IPS_ALERT);
    SafetyLP.fill(dev, "SCOPE_SAFETY", "Scope safety", "Safety", IPS_ALERT);

    // --- ESP32 TCP command port ----------------------------------------------------------
    EspTP[0].fill("HOST", "ESP32 host (empty = auto)", "");
    EspTP.fill(dev, "ESP_HOST", "ESP32 address", "Encoder", IP_RW, 0, IPS_IDLE);
    EspNP[E_PORT].fill("TCP_PORT", "Command port", "%.0f", 1, 65535, 1, 2348);
    EspNP[E_TIMEOUT].fill("TIMEOUT_MS", "Timeout (ms)", "%.0f", 200, 10000, 100, 1500);
    EspNP.fill(dev, "ESP_COMMAND_PORT", "ESP32 command port", "Encoder", IP_RW, 0, IPS_IDLE);
    EspSetNP[0].fill("VALUE_UM", "Set ESP32 encoder to (um)", "%.0f", -1e7, 1e8, 1, 0);
    EspSetNP.fill(dev, "ESP_SET_POSITION", "Set ESP32 position", "Encoder", IP_RW, 0, IPS_IDLE);
    SyncPushSP[0].fill("SYNC_OFFSET", "Sync changes the offset", ISS_ON);
    SyncPushSP[1].fill("SYNC_ESP32", "Sync sets the ESP32 encoder", ISS_OFF);
    SyncPushSP.fill(dev, "SYNC_MODE", "Sync mode", "Encoder", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

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
    RestoreSP[1].fill("RESTORE_ON", "Restore last position (offset)", ISS_OFF);
    RestoreSP[2].fill("RESTORE_ESP", "Push last position to ESP32", ISS_OFF);
    RestoreSP.fill(dev, "RESTORE_MODE", "Position at connect", "Encoder", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Travel range comes from LOOP_PARAMS (defaults 500 .. 6000 um; config is loaded later)
    applyLimits();
    setAbsValue(LimitsNP[LIM_MIN].getValue());

    return true;
}

void EncoderFocuser::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);

    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    defineProperty(UdpNP);
    defineProperty(GpioNP);
    defineProperty(MotorNP);
    defineProperty(LoopNP);
    defineProperty(LimitsNP);
    defineProperty(PosStateNP);
    defineProperty(EnablePolSP);
    defineProperty(DirPolSP);
    defineProperty(RampSP);
    defineProperty(RestoreSP);
    defineProperty(ImuNP);
    defineProperty(ImuCheckSP);
    defineProperty(ImuCaptureSP);
    defineProperty(EspTP);
    defineProperty(EspNP);
    defineProperty(EspSetNP);
    defineProperty(SyncPushSP);

    if (!configLoaded_)
    {
        configLoaded_ = true;
        loadConfig(true, UdpNP.getName());
        loadConfig(true, GpioNP.getName());
        loadConfig(true, MotorNP.getName());
        loadConfig(true, LoopNP.getName());
        loadConfig(true, LimitsNP.getName());
        loadConfig(true, PosStateNP.getName());
        loadConfig(true, EnablePolSP.getName());
        loadConfig(true, DirPolSP.getName());
        loadConfig(true, RampSP.getName());
        loadConfig(true, RestoreSP.getName());
        loadConfig(true, ImuNP.getName());
        loadConfig(true, ImuCheckSP.getName());
        loadConfig(true, EspTP.getName());
        loadConfig(true, EspNP.getName());
        loadConfig(true, SyncPushSP.getName());
        refreshCfg();
        applyLimits();
    }
}

bool EncoderFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        applyLimits();      // the base class may have reset FOCUS_MAX; our saved value wins
        applyAbs();
        applyMax();
        defineProperty(StatusNP);
        defineProperty(ImuAnglesNP);
        defineProperty(SafetyLP);
    }
    else
    {
        deleteProperty(StatusNP.getName());
        deleteProperty(ImuAnglesNP.getName());
        deleteProperty(SafetyLP.getName());
    }
    return true;
}

bool EncoderFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);
    UdpNP.save(fp);
    GpioNP.save(fp);
    MotorNP.save(fp);
    LoopNP.save(fp);
    LimitsNP.save(fp);
    PosStateNP.save(fp);
    EnablePolSP.save(fp);
    DirPolSP.save(fp);
    RampSP.save(fp);
    RestoreSP.save(fp);
    ImuNP.save(fp);
    ImuCheckSP.save(fp);
    EspTP.save(fp);
    EspNP.save(fp);
    SyncPushSP.save(fp);
    return true;
}

// Copy property values into the plain struct used by the motion thread.
void EncoderFocuser::refreshCfg()
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
    c.minPos = static_cast<int>(LimitsNP[LIM_MIN].getValue());
    c.maxPos = static_cast<int>(LimitsNP[LIM_MAX].getValue());
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
bool EncoderFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
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
            return true;
        }
        if (handle(LimitsNP))
        {
            if (LimitsNP[LIM_MIN].getValue() >= LimitsNP[LIM_MAX].getValue())
            {
                LOG_ERROR("Minimum position must be below the maximum position.");
                LimitsNP.setState(IPS_ALERT);
                LimitsNP.apply();
                return true;
            }
            applyLimits();
            refreshCfg();
            applyAbs();
            applyMax();
            return true;
        }
        if (handle(ImuNP))
        {
            if (isConnected())
                restartImuUdp();
            return true;
        }
        if (handle(EspNP))
            return true;
        if (EspSetNP.isNameMatch(name))
        {
            if (moving_)
            {
                LOG_ERROR("Cannot set the ESP32 position while moving.");
                EspSetNP.setState(IPS_ALERT);
                EspSetNP.apply();
                return true;
            }
            EspSetNP.update(values, names, n);
            std::string err;
            bool ok = espPush(static_cast<int>(EspSetNP[0].getValue()), err);
            if (!ok)
                LOGF_ERROR("Setting the ESP32 position failed: %s", err.c_str());
            EspSetNP.setState(ok ? IPS_OK : IPS_ALERT);
            EspSetNP.apply();
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

bool EncoderFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
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
        if (handle(ImuCheckSP) || handle(SyncPushSP))
            return true;
        if (ImuCaptureSP.isNameMatch(name))
        {
            if (imuValid_)
            {
                ImuNP[I_YAW_C].setValue(imuYaw_.load());
                ImuNP[I_PITCH_C].setValue(imuPitch_.load());
                ImuNP[I_ROLL_C].setValue(imuRoll_.load());
                ImuNP.setState(IPS_OK);
                ImuNP.apply();
                LOGF_INFO("Safe zone centre set to yaw %.1f, pitch %.1f, roll %.1f (press Save to keep).",
                          imuYaw_.load(), imuPitch_.load(), imuRoll_.load());
                ImuCaptureSP.setState(IPS_OK);
            }
            else
            {
                LOG_ERROR("No IMU data received yet.");
                ImuCaptureSP.setState(IPS_ALERT);
            }
            ImuCaptureSP[0].setState(ISS_OFF);
            ImuCaptureSP.apply();
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
bool EncoderFocuser::Connect()
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
    if (!gpio_.request(chip, offsets, initial, 3, "indi_encoder_focuser", err))
    {
        LOGF_ERROR("GPIO setup failed: %s", err.c_str());
        return false;
    }
    motorEnabled_ = false;

    // Encoder bookkeeping
    encValid_ = false;
    lastRxNs_ = 0;
    offsetUm_ = static_cast<int>(PosStateNP[P_OFFSET].getValue());
    restorePending_ = (RestoreSP.findOnSwitchIndex() >= 1);
    restoreRetryNs_ = 0;
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
    {
        int imuPort = static_cast<int>(ImuNP[I_UDP_PORT].getValue());
        if (imuPort > 0 && imuPort != static_cast<int>(UdpNP[U_PORT].getValue()))
            LOGF_INFO("Listening for IMU data on UDP port %d.", imuPort);
        else
            LOG_INFO("IMU data is expected on the encoder port.");
    }
    if (restorePending_)
        LOG_INFO("Waiting for first encoder reading to restore the last saved position.");

    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool EncoderFocuser::Disconnect()
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
bool EncoderFocuser::openUdp(UdpIn &u, int port, bool isMain, std::string &err)
{
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
    u.fd = fd;
    u.run = true;
    u.th = std::thread(&EncoderFocuser::udpLoop, this, &u, isMain);
    return true;
}

void EncoderFocuser::closeUdp(UdpIn &u)
{
    u.run = false;
    if (u.th.joinable())
        u.th.join();
    if (u.fd >= 0)
    {
        close(u.fd);
        u.fd = -1;
    }
}

// Encoder port (always) and IMU port (when set and different from the encoder port).
// Both sockets accept both line types, so a single-port setup keeps working.
bool EncoderFocuser::startUdp(std::string &err)
{
    int port = static_cast<int>(UdpNP[U_PORT].getValue());
    if (!openUdp(udpMain_, port, true, err))
        return false;
    int imuPort = static_cast<int>(ImuNP[I_UDP_PORT].getValue());
    if (imuPort > 0 && imuPort != port)
    {
        if (!openUdp(udpImu_, imuPort, false, err))
        {
            closeUdp(udpMain_);
            return false;
        }
    }
    return true;
}

void EncoderFocuser::stopUdp()
{
    closeUdp(udpMain_);
    closeUdp(udpImu_);
}

bool EncoderFocuser::restartImuUdp()
{
    closeUdp(udpImu_);
    int port = static_cast<int>(UdpNP[U_PORT].getValue());
    int imuPort = static_cast<int>(ImuNP[I_UDP_PORT].getValue());
    if (imuPort <= 0 || imuPort == port)
    {
        LOG_INFO("IMU data is expected on the encoder port.");
        return true;
    }
    std::string err;
    if (!openUdp(udpImu_, imuPort, false, err))
    {
        LOGF_ERROR("Cannot open the IMU UDP port: %s", err.c_str());
        return false;
    }
    LOGF_INFO("Listening for IMU data on UDP port %d.", imuPort);
    return true;
}

void EncoderFocuser::udpLoop(UdpIn *u, bool isMain)
{
    char buf[2048];
    while (u->run)
    {
        pollfd pfd;
        pfd.fd = u->fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int r = poll(&pfd, 1, 200);
        if (r <= 0)
            continue;
        sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        ssize_t len = recvfrom(u->fd, buf, sizeof(buf) - 1, 0, reinterpret_cast<sockaddr *>(&from), &fromLen);
        if (len <= 0)
            continue;
        if (isMain)
            lastSenderIp_ = from.sin_addr.s_addr; // address used for the ESP32 command port
        buf[len] = '\0';
        parseDatagram(buf, static_cast<size_t>(len));
    }
}

// A datagram may hold one or several lines. "ypr..." lines (IMU) are ignored; an encoder
// line is a plain (optionally signed) integer in encoder counts. Last valid line wins.
void EncoderFocuser::parseDatagram(const char *buf, size_t len)
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
        {
            // "ypr <yaw> <pitch> <roll>" in degrees, separators: space, tab, comma, colon
            const char *q = p + 3;
            double v[3];
            int got = 0;
            while (got < 3)
            {
                while (*q == ' ' || *q == '\t' || *q == ',' || *q == ':')
                    q++;
                char *e = nullptr;
                double d = strtod(q, &e);
                if (e == q)
                    break;
                v[got++] = d;
                q = e;
            }
            if (got == 3)
                setAngles(v[0], v[1], v[2]);
            continue;
        }
        if (*p == 'w')
        {
            // quaternion: w<v>wa<v>ab<v>bc<v>c
            double w, x, y, z;
            if (parseQuaternion(p, w, x, y, z))
            {
                double n = std::sqrt(w * w + x * x + y * y + z * z);
                if (n > 1e-9)
                {
                    w /= n; x /= n; y /= n; z /= n;
                    // MPU6050/DMP convention (Jeff Rowberg's dmpGetYawPitchRoll)
                    double gx = 2 * (x * z - w * y);
                    double gy = 2 * (w * x + y * z);
                    double gz = w * w - x * x - y * y + z * z;
                    const double r2d = 180.0 / M_PI;
                    double yaw = std::atan2(2 * x * y - 2 * w * z, 2 * w * w + 2 * x * x - 1) * r2d;
                    double pitch = std::atan(gx / std::sqrt(gy * gy + gz * gz)) * r2d;
                    double roll = std::atan(gy / std::sqrt(gx * gx + gz * gz)) * r2d;
                    setAngles(yaw, pitch, roll);
                }
            }
            continue;
        }

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

bool EncoderFocuser::parseQuaternion(const char *p, double &w, double &x, double &y, double &z)
{
    char *e = nullptr;
    const char *q = p;
    if (*q++ != 'w') return false;
    w = strtod(q, &e); if (e == q || *e != 'w') return false;
    q = e + 1;
    if (*q++ != 'a') return false;
    x = strtod(q, &e); if (e == q || *e != 'a') return false;
    q = e + 1;
    if (*q++ != 'b') return false;
    y = strtod(q, &e); if (e == q || *e != 'b') return false;
    q = e + 1;
    if (*q++ != 'c') return false;
    z = strtod(q, &e); if (e == q || *e != 'c') return false;
    return true;
}

void EncoderFocuser::setAngles(double yaw, double pitch, double roll)
{
    imuYaw_ = yaw;
    imuPitch_ = pitch;
    imuRoll_ = roll;
    lastImuNs_ = nowNs();
    imuValid_ = true;
}

static double wrapDiff(double a, double c)
{
    return std::fmod(a - c + 540.0, 360.0) - 180.0;
}

// Safe = fresh IMU data AND at least one axis checked AND every checked axis within centre +/- tol,
// continuously for SAFE_HOLD_MS. Becomes unsafe immediately. Published as light SCOPE_SAFETY/SAFE,
// refreshed at least once a second so that snooping drivers can detect a dead publisher.
void EncoderFocuser::updateSafety()
{
    int64_t now = nowNs();
    bool fresh = imuValid_ && ((now - lastImuNs_.load()) / 1000000 < ImuNP[I_TIMEOUT_MS].getValue());

    bool any = false, inRange = fresh;
    const double ang[3] = {imuYaw_.load(), imuPitch_.load(), imuRoll_.load()};
    for (int a = 0; a < 3; a++)
    {
        if (ImuCheckSP[a].getState() != ISS_ON)
            continue;
        any = true;
        if (std::fabs(wrapDiff(ang[a], ImuNP[I_YAW_C + a].getValue())) > ImuNP[I_YAW_T + a].getValue())
            inRange = false;
    }
    if (!any)
        inRange = false;

    if (!inRange)
    {
        imuSafe_ = false;
        imuSafeSinceNs_ = 0;
    }
    else if (!imuSafe_)
    {
        if (imuSafeSinceNs_ == 0)
            imuSafeSinceNs_ = now;
        if ((now - imuSafeSinceNs_) / 1000000 >= ImuNP[I_HOLD_MS].getValue())
            imuSafe_ = true;
    }

    bool changed = (imuSafe_ != lastSafetyPub_);
    if (changed)
    {
        if (imuSafe_)
            LOG_INFO("IMU: scope is in the safe zone.");
        else if (!any)
            LOG_WARN("IMU: no axis selected for the safety check; reporting NOT safe.");
        else if (!fresh)
            LOG_WARN("IMU: no recent data; reporting NOT safe.");
        else
            LOGF_WARN("IMU: scope outside the safe zone (yaw %.1f, pitch %.1f, roll %.1f).", ang[0], ang[1], ang[2]);
    }

    if (changed || (now - lastSafetyPubNs_) >= 1000000000LL)
    {
        lastSafetyPub_ = imuSafe_;
        lastSafetyPubNs_ = now;
        IPState st = imuSafe_ ? IPS_OK : IPS_ALERT;
        SafetyLP[0].setState(st);
        SafetyLP.setState(st);
        SafetyLP.apply();
        ImuAnglesNP[0].setValue(ang[0]);
        ImuAnglesNP[1].setValue(ang[1]);
        ImuAnglesNP[2].setValue(ang[2]);
        ImuAnglesNP.setState(fresh ? IPS_OK : IPS_ALERT);
        ImuAnglesNP.apply();
    }
}

// ---- ESP32 command port ----------------------------------------------------------------
// Protocol (see the ESP32 sketch): a line "s <value>" sets the encoder to <value> um and is
// answered with "OK <value>". The session is closed with "q".
bool EncoderFocuser::espCommandSet(int um, std::string &err)
{
    std::string host = EspTP[0].getText();
    if (host.empty())
    {
        uint32_t ip = lastSenderIp_.load();
        if (ip == 0)
        {
            err = "no ESP32 address: fill ESP_HOST or wait for UDP data";
            return false;
        }
        in_addr a;
        a.s_addr = ip;
        char b[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &a, b, sizeof(b));
        host = b;
    }
    const int port = static_cast<int>(EspNP[E_PORT].getValue());
    const int timeoutMs = static_cast<int>(EspNP[E_TIMEOUT].getValue());
    const int64_t deadline = nowNs() + static_cast<int64_t>(timeoutMs) * 1000000LL;
    auto remaining = [&]() -> int
    {
        int64_t r = (deadline - nowNs()) / 1000000;
        return r < 0 ? 0 : static_cast<int>(r);
    };

    addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo *res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || res == nullptr)
    {
        err = "cannot resolve " + host;
        return false;
    }
    int fd = socket(res->ai_family, res->ai_socktype | SOCK_CLOEXEC, res->ai_protocol);
    if (fd < 0)
    {
        err = strerror(errno);
        freeaddrinfo(res);
        return false;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    int rc = connect(fd, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);
    if (rc < 0 && errno != EINPROGRESS)
    {
        err = std::string("connect ") + host + ":" + std::to_string(port) + ": " + strerror(errno);
        close(fd);
        return false;
    }
    if (rc < 0)
    {
        pollfd p{fd, POLLOUT, 0};
        int soerr = 0;
        socklen_t sl = sizeof(soerr);
        if (poll(&p, 1, remaining()) <= 0 || getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &sl) != 0 || soerr != 0)
        {
            err = "connect " + host + ":" + std::to_string(port) + " failed or timed out";
            close(fd);
            return false;
        }
    }

    std::string cmd = "s " + std::to_string(um) + "\n";
    if (send(fd, cmd.data(), cmd.size(), MSG_NOSIGNAL) != static_cast<ssize_t>(cmd.size()))
    {
        err = "send failed";
        close(fd);
        return false;
    }

    // Read lines until "OK ..." or "ERR ..." (the greeting banner is skipped).
    std::string acc;
    bool ok = false, done = false;
    while (!done)
    {
        pollfd p{fd, POLLIN, 0};
        int pr = poll(&p, 1, remaining());
        if (pr <= 0)
        {
            err = "no answer from the ESP32";
            break;
        }
        char buf[256];
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0)
        {
            err = "connection closed by the ESP32";
            break;
        }
        acc.append(buf, static_cast<size_t>(n));
        size_t pos;
        while (!done && (pos = acc.find('\n')) != std::string::npos)
        {
            std::string line = acc.substr(0, pos);
            acc.erase(0, pos + 1);
            while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
                line.pop_back();
            if (line.compare(0, 2, "OK") == 0)
            {
                ok = true;
                done = true;
            }
            else if (line.compare(0, 3, "ERR") == 0)
            {
                err = "ESP32 replied: " + line;
                done = true;
            }
        }
    }
    send(fd, "q\n", 2, MSG_NOSIGNAL);
    close(fd);
    return ok;
}

// Set the ESP32 encoder to `um` and wait until a datagram with the new value has arrived.
// The ESP32 then holds the position itself, so no software offset is needed any more.
bool EncoderFocuser::espPush(int um, std::string &err)
{
    int64_t t0 = nowNs();
    if (!espCommandSet(um, err))
        return false;
    offsetUm_ = 0;
    for (int i = 0; i < 100 && lastRxNs_.load() <= t0; i++)
        usleep(10000); // up to 1 s for a fresh reading
    if (lastRxNs_.load() <= t0)
    {
        err = "command accepted but no fresh encoder reading arrived";
        return false;
    }
    LOGF_INFO("ESP32 encoder set to %d um (reading now %d um).", um, positionUm());
    persistPosition();
    return true;
}

int EncoderFocuser::positionUm() const
{
    return encRaw_.load() * scaleUm_.load() + offsetUm_.load();
}

int64_t EncoderFocuser::dataAgeMs() const
{
    if (!encValid_)
        return INT64_C(1000000000);
    return (nowNs() - lastRxNs_.load()) / 1000000;
}

// =====================================================================================
//  Motor control
// =====================================================================================
void EncoderFocuser::setEnable(bool on, bool activeLow)
{
    int level = on ? (activeLow ? 0 : 1) : (activeLow ? 1 : 0);
    gpio_.set(IDX_ENABLE, level);
    motorEnabled_ = on;
}

int64_t EncoderFocuser::nowNs()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Sleep for the bulk of the interval, spin for the last ~100 us.
void EncoderFocuser::waitUntil(int64_t target)
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

void EncoderFocuser::stopMotion()
{
    abortReq_ = true;
    if (moveThread_.joinable())
        moveThread_.join();
    abortReq_ = false;
    moving_ = false;
}

IPState EncoderFocuser::startMove(int target)
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
    moveThread_ = std::thread(&EncoderFocuser::motionThread, this, target, c);
    return IPS_BUSY;
}

// Closed-loop move: step toward the target, re-evaluating the encoder before every pulse.
void EncoderFocuser::motionThread(int target, Cfg c)
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
    measuredSlackUm_ = -1;

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
        if (raw != lastRaw)
        {
            if (!changed && slackUm > 0)
                measuredSlackUm_ = static_cast<int>(sinceChange / k); // travel before the encoder reacted
            lastRaw = raw;
            sinceChange = 0;
            changed = true;
        }
        else
            sinceChange++;
        double limit = (c.stallMarginUm + (changed ? 0.0 : slackUm)) * k;
        if (sinceChange > limit)
        {
            stallTravelUm_ = static_cast<int>(sinceChange / k);
            stallAllowUm_ = static_cast<int>(limit / k);
            res = MoveResult::Stall;
            break;
        }

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
void EncoderFocuser::finishMove()
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
            LOGF_ERROR("Stall: motor travelled ~%d um without encoder change (allowance %d um). Stopped at %d um. "
                       "If this follows a direction change, raise BACKLASH_IN/OUT_UM or STALL_MARGIN_UM.",
                       stallTravelUm_.load(), stallAllowUm_.load(), pos);
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
    if (measuredSlackUm_ >= 0)
        LOGF_INFO("Backlash measured on this reversal: ~%d um of motor travel before the encoder moved.",
                  measuredSlackUm_.load());
    setAbsValue(pos);
    setAbsState(st);
    setRelState(st);
    applyAbs();
    applyRel();
    persistPosition();
}

void EncoderFocuser::persistPosition()
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
IPState EncoderFocuser::MoveAbsFocuser(uint32_t target)
{
    return startMove(static_cast<int>(target));
}

IPState EncoderFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // INDI convention: outward = position increases.
    int delta = (dir == FOCUS_OUTWARD) ? static_cast<int>(ticks) : -static_cast<int>(ticks);
    int target = positionUm() + delta;
    return startMove(target);
}

bool EncoderFocuser::AbortFocuser()
{
    abortReq_ = true; // the motion thread stops at its next step; TimerHit reports it
    return true;
}

bool EncoderFocuser::SyncFocuser(uint32_t ticks)
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
    if (SyncPushSP.findOnSwitchIndex() == 1)
    {
        std::string err;
        if (!espPush(static_cast<int>(ticks), err))
        {
            LOGF_ERROR("Sync via ESP32 failed: %s", err.c_str());
            return false;
        }
        setAbsValue(positionUm());
        applyAbs();
        LOGF_INFO("Synced through the ESP32: position is now %d um.", positionUm());
        return true;
    }
    offsetUm_ = static_cast<int>(ticks) - encRaw_.load() * scaleUm_.load();
    setAbsValue(ticks);
    applyAbs();
    persistPosition();
    LOGF_INFO("Synced: position is now %u um (offset %d um).", ticks, offsetUm_.load());
    return true;
}

bool EncoderFocuser::SetFocuserMaxPosition(uint32_t ticks)
{
    // The client changed FOCUS_MAX: keep the driver's own (persisted) value in step.
    LimitsNP[LIM_MAX].setValue(ticks);
    LoopNP.apply();
    applyLimits();
    refreshCfg();
    return true;
}

void EncoderFocuser::applyLimits()
{
    setMaxPos(LimitsNP[LIM_MAX].getValue());
    setAbsLimits(LimitsNP[LIM_MIN].getValue(), LimitsNP[LIM_MAX].getValue());
}

// =====================================================================================
//  Periodic housekeeping (100 ms)
// =====================================================================================
void EncoderFocuser::TimerHit()
{
    if (!isConnected())
        return;

    tick_++;

    // One-time position restore once the first reading is available.
    if (restorePending_ && encValid_)
    {
        if (RestoreSP.findOnSwitchIndex() == 2)
        {
            // Push the last saved position into the ESP32; retry every 5 s until it works.
            if (nowNs() >= restoreRetryNs_)
            {
                std::string err;
                int last = static_cast<int>(PosStateNP[P_LAST].getValue());
                if (espPush(last, err))
                {
                    restorePending_ = false;
                    LOGF_INFO("Pushed last position %d um to the ESP32 (now reads %d um).", last, positionUm());
                    persistPosition();
                }
                else
                {
                    LOGF_ERROR("Cannot push the last position to the ESP32: %s (retrying; moves are blocked).",
                               err.c_str());
                    restoreRetryNs_ = nowNs() + 5000000000LL;
                }
            }
        }
        else
        {
            offsetUm_ = static_cast<int>(PosStateNP[P_LAST].getValue()) - encRaw_.load() * scaleUm_.load();
            restorePending_ = false;
            LOGF_INFO("Restored position: %d um (offset %d um).", positionUm(), offsetUm_.load());
            persistPosition();
        }
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

    updateSafety();
    SetTimer(getCurrentPollingPeriod());
}

// =====================================================================================
//  Access to FocuserInterface properties (legacy structs vs. INDI::PropertyNumber)
// =====================================================================================
#if EF_FOCUS_WRAPPED
void EncoderFocuser::setAbsValue(double v) { FocusAbsPosNP[0].setValue(v); }
void EncoderFocuser::setAbsState(IPState s) { FocusAbsPosNP.setState(s); }
void EncoderFocuser::applyAbs() { FocusAbsPosNP.apply(); }
void EncoderFocuser::setRelState(IPState s) { FocusRelPosNP.setState(s); }
void EncoderFocuser::applyRel() { FocusRelPosNP.apply(); }
void EncoderFocuser::setAbsLimits(double mn, double mx)
{
    FocusAbsPosNP[0].setMin(mn);
    FocusAbsPosNP[0].setMax(mx);
    FocusAbsPosNP.updateMinMax();
}
double EncoderFocuser::getMaxPos() { return FocusMaxPosNP[0].getValue(); }
void EncoderFocuser::setMaxPos(double v) { FocusMaxPosNP[0].setValue(v); }
void EncoderFocuser::applyMax() { FocusMaxPosNP.apply(); }
#else
void EncoderFocuser::setAbsValue(double v) { FocusAbsPosN[0].value = v; }
void EncoderFocuser::setAbsState(IPState s) { FocusAbsPosNP.s = s; }
void EncoderFocuser::applyAbs() { IDSetNumber(&FocusAbsPosNP, nullptr); }
void EncoderFocuser::setRelState(IPState s) { FocusRelPosNP.s = s; }
void EncoderFocuser::applyRel() { IDSetNumber(&FocusRelPosNP, nullptr); }
void EncoderFocuser::setAbsLimits(double mn, double mx)
{
    FocusAbsPosN[0].min = mn;
    FocusAbsPosN[0].max = mx;
    IUUpdateMinMax(&FocusAbsPosNP);
}
double EncoderFocuser::getMaxPos() { return FocusMaxPosN[0].value; }
void EncoderFocuser::setMaxPos(double v) { FocusMaxPosN[0].value = v; }
void EncoderFocuser::applyMax() { IDSetNumber(&FocusMaxPosNP, nullptr); }
#endif


bool EncoderFocuser::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && EspTP.isNameMatch(name))
    {
        bool ok = EspTP.update(texts, names, n);
        EspTP.setState(ok ? IPS_OK : IPS_ALERT);
        EspTP.apply();
        return true;
    }
    return INDI::Focuser::ISNewText(dev, name, texts, names, n);
}
