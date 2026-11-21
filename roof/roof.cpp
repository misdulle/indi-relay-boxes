#include "roof.h"
#include "../common/http_client.h"

#include <indilogger.h>
#include <lilxml.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <regex>
#include <algorithm>

static std::unique_ptr<RollOffRoof> roof(new RollOffRoof());

namespace
{
constexpr int64_t kImuStaleNs = 5000000000LL;   // focuser refreshes SCOPE_SAFETY at least every 1 s
constexpr int64_t kPowerRetryNs = 1000000000LL;
constexpr int64_t kStatusEveryNs = 2000000000LL;
}

int64_t RollOffRoof::nowNs()
{
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

RollOffRoof::RollOffRoof()
{
    setVersion(1, 0);
    SetDomeCapability(DOME_CAN_ABORT | DOME_CAN_PARK | DOME_HAS_SHUTTER);
    setDomeConnection(CONNECTION_NONE);
    setDefaultPollingPeriod(100);
}

RollOffRoof::~RollOffRoof()
{
    {
        std::lock_guard<std::mutex> lk(qm_);
        workerRun_ = false;
    }
    qcv_.notify_all();
    if (worker_.joinable())
        worker_.join();
}

const char *RollOffRoof::getDefaultName()
{
    return "eulRoof";
}

bool RollOffRoof::initProperties()
{
    INDI::Dome::initProperties();
    SetParkDataType(PARK_NONE);
    const char *dev = getDeviceName();

    RelayHostTP[0].fill("HOST", "16-port box IP / host", "192.168.1.28");
    RelayHostTP.fill(dev, "ROOF_RELAY_BOX", "Roof control relays", "Setup", IP_RW, 0, IPS_IDLE);

    RelayNP[R_PORT].fill("TCP_PORT", "TCP port", "%.0f", 1, 65535, 1, 80);
    RelayNP[R_ADDR].fill("ADDR", "Base address (ADDR)", "%.0f", 0, 99, 1, 30);
    RelayNP[R_TIMEOUT].fill("TIMEOUT_MS", "HTTP timeout (ms)", "%.0f", 200, 30000, 100, 2000);
    RelayNP[R_STOP].fill("RELAY_STOP", "Relay STOP", "%.0f", 1, 16, 1, 1);
    RelayNP[R_OPEN].fill("RELAY_OPEN", "Relay OPEN", "%.0f", 1, 16, 1, 2);
    RelayNP[R_CLOSE].fill("RELAY_CLOSE", "Relay CLOSE", "%.0f", 1, 16, 1, 3);
    RelayNP[R_PULSE].fill("PULSE_MS", "Pulse length (ms)", "%.0f", 100, 5000, 50, 500);
    RelayNP.fill(dev, "ROOF_RELAY_PARAMS", "Relay parameters", "Setup", IP_RW, 0, IPS_IDLE);

    PulsePolSP[0].fill("PULSE_RELAY_ON", "Relay ON during pulse (normally open)", ISS_ON);
    PulsePolSP[1].fill("PULSE_RELAY_OFF", "Relay OFF during pulse (normally closed)", ISS_OFF);
    PulsePolSP.fill(dev, "PULSE_POLARITY", "Pulse polarity", "Setup", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    PowerHostTP[H_HOST].fill("HOST", "KMTronic IP / host", "192.168.1.23");
    PowerHostTP[H_USER].fill("USER", "User (empty = no auth)", "admin");
    PowerHostTP[H_PASS].fill("PASSWORD", "Password", "admin");
    PowerHostTP.fill(dev, "ROOF_POWER_BOX", "Motor power relay box", "Setup", IP_RW, 0, IPS_IDLE);

    PowerNP[P_PORT].fill("TCP_PORT", "TCP port", "%.0f", 1, 65535, 1, 80);
    PowerNP[P_TIMEOUT].fill("TIMEOUT_MS", "HTTP timeout (ms)", "%.0f", 200, 30000, 100, 2000);
    PowerNP[P_RELAY].fill("MOTOR_RELAY", "Motor power relay (1-8)", "%.0f", 1, 8, 1, 4);
    PowerNP.fill(dev, "ROOF_POWER_PARAMS", "Motor power parameters", "Setup", IP_RW, 0, IPS_IDLE);

    PowerStyleSP[0].fill("STYLE_AUTO", "Auto (FF, fall back to toggle)", ISS_ON);
    PowerStyleSP[1].fill("STYLE_FF", "/FF0N01 on, /FF0N00 off", ISS_OFF);
    PowerStyleSP[2].fill("STYLE_CGI", "/relays.cgi?relay=N (toggle)", ISS_OFF);
    PowerStyleSP.fill(dev, "POWER_COMMAND_STYLE", "Motor power command style", "Setup", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    SourcesTP[SRC_MOUNT].fill("MOUNT_DEVICE", "Mount device (TELESCOPE_PARK)", "EQMod Mount");
    SourcesTP[SRC_FOCUSER].fill("FOCUSER_DEVICE", "Focuser device (SCOPE_SAFETY)", "RPi Encoder Focuser");
    SourcesTP.fill(dev, "SAFETY_SOURCES", "Safety sources", "Safety", IP_RW, 0, IPS_IDLE);

    PolicySP[POL_PARKED].fill("REQUIRE_PARKED", "Require mount parked", ISS_ON);
    PolicySP[POL_IMU].fill("REQUIRE_IMU_SAFE", "Require IMU safe", ISS_ON);
    PolicySP[POL_CUT].fill("CUT_POWER_WHEN_UNSAFE", "Motor power only when safe", ISS_ON);
    PolicySP[POL_OVERRIDE].fill("OVERRIDE", "OVERRIDE: ignore all safety checks", ISS_OFF);
    PolicySP.fill(dev, "SAFETY_POLICY", "Safety policy", "Safety", IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    TravelNP[T_OPEN].fill("OPEN_TIME_S", "Opening time (s)", "%.0f", 5, 600, 1, 60);
    TravelNP[T_CLOSE].fill("CLOSE_TIME_S", "Closing time (s)", "%.0f", 5, 600, 1, 60);
    TravelNP.fill(dev, "ROOF_TRAVEL", "Travel time", "Setup", IP_RW, 0, IPS_IDLE);

    MarkSP[MARK_OPEN].fill("MARK_OPEN", "Roof is open", ISS_OFF);
    MarkSP[MARK_CLOSED].fill("MARK_CLOSED", "Roof is closed", ISS_OFF);
    MarkSP[MARK_UNKNOWN].fill("MARK_UNKNOWN", "Roof state unknown", ISS_ON);
    MarkSP.fill(dev, "ROOF_STATE_MARK", "Roof state (set manually / remembered)", "Main Control", IP_RW, ISR_1OFMANY, 0,
                IPS_IDLE);

    StatusLP[ST_PARKED].fill("MOUNT_PARKED", "Mount parked", IPS_IDLE);
    StatusLP[ST_IMU].fill("IMU_SAFE", "Scope in safe position", IPS_IDLE);
    StatusLP[ST_POWER].fill("MOTOR_POWER", "Motor power on", IPS_IDLE);
    StatusLP[ST_ALLOWED].fill("MOVE_ALLOWED", "Movement allowed", IPS_IDLE);
    StatusLP.fill(dev, "ROOF_SAFETY_STATUS", "Safety status", "Safety", IPS_IDLE);
    return true;
}

void RollOffRoof::ISGetProperties(const char *dev)
{
    INDI::Dome::ISGetProperties(dev);
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    defineProperty(RelayHostTP);
    defineProperty(RelayNP);
    defineProperty(PulsePolSP);
    defineProperty(PowerHostTP);
    defineProperty(PowerNP);
    defineProperty(PowerStyleSP);
    defineProperty(SourcesTP);
    defineProperty(PolicySP);
    defineProperty(TravelNP);
    defineProperty(MarkSP);

    if (!configLoaded_)
    {
        configLoaded_ = true;
        loadConfig(true, RelayHostTP.getName());
        loadConfig(true, RelayNP.getName());
        loadConfig(true, PulsePolSP.getName());
        loadConfig(true, PowerHostTP.getName());
        loadConfig(true, PowerNP.getName());
        loadConfig(true, PowerStyleSP.getName());
        kmStyle_ = std::max(0, PowerStyleSP.findOnSwitchIndex());
        loadConfig(true, SourcesTP.getName());
        loadConfig(true, PolicySP.getName());
        loadConfig(true, TravelNP.getName());
        loadConfig(true, MarkSP.getName());
        resnoop();
    }
}

bool RollOffRoof::updateProperties()
{
    INDI::Dome::updateProperties();
    if (isConnected())
    {
        defineProperty(StatusLP);
        if (InitPark())
            SetAxis1ParkDefault(0);
        else
        {
            SetAxis1Park(0);
            SetAxis1ParkDefault(0);
        }
        applyMark(MarkSP.findOnSwitchIndex() < 0 ? MARK_UNKNOWN : MarkSP.findOnSwitchIndex(), false);
    }
    else
        deleteProperty(StatusLP.getName());
    return true;
}

bool RollOffRoof::saveConfigItems(FILE *fp)
{
    INDI::Dome::saveConfigItems(fp);
    RelayHostTP.save(fp);
    RelayNP.save(fp);
    PulsePolSP.save(fp);
    PowerHostTP.save(fp);  // password is stored in clear text
    PowerNP.save(fp);
    PowerStyleSP.save(fp);
    SourcesTP.save(fp);
    PolicySP.save(fp);
    TravelNP.save(fp);
    MarkSP.save(fp);
    return true;
}

bool RollOffRoof::policy(int idx) const
{
    return PolicySP[idx].getState() == ISS_ON;
}

// =====================================================================================
//  Worker thread: all HTTP traffic happens here so a slow/dead box never blocks INDI
// =====================================================================================
RollOffRoof::Cfg RollOffRoof::makeCfg()
{
    Cfg c;
    c.relayHost = RelayHostTP[0].getText();
    c.relayPort = static_cast<int>(RelayNP[R_PORT].getValue());
    c.relayAddr = static_cast<int>(RelayNP[R_ADDR].getValue());
    c.relayTimeout = static_cast<int>(RelayNP[R_TIMEOUT].getValue());
    c.relayStop = static_cast<int>(RelayNP[R_STOP].getValue());
    c.relayOpen = static_cast<int>(RelayNP[R_OPEN].getValue());
    c.relayClose = static_cast<int>(RelayNP[R_CLOSE].getValue());
    c.pulseMs = static_cast<int>(RelayNP[R_PULSE].getValue());
    c.pulseRelayOn = (PulsePolSP.findOnSwitchIndex() != 1);
    c.powerHost = PowerHostTP[H_HOST].getText();
    c.powerUser = PowerHostTP[H_USER].getText();
    c.powerPass = PowerHostTP[H_PASS].getText();
    c.powerPort = static_cast<int>(PowerNP[P_PORT].getValue());
    c.powerTimeout = static_cast<int>(PowerNP[P_TIMEOUT].getValue());
    c.motorRelay = static_cast<int>(PowerNP[P_RELAY].getValue());
    return c;
}

bool RollOffRoof::httpGet(const std::string &host, int port, const std::string &path, int timeoutMs,
                          const std::string &user, const std::string &pass, std::string &body, std::string &err)
{
    int status = 0;
    std::string hdr = httpc::basicAuthHeader(user, pass);
    if (!httpc::get(host, port, path, timeoutMs, status, body, err, hdr))
        return false;
    if (status != 200)
    {
        err = "HTTP status " + std::to_string(status) + " for " + path;
        return false;
    }
    return true;
}

bool RollOffRoof::readMotorRelay(const Cfg &c, int &state, std::string &err)
{
    std::string body;
    if (!httpGet(c.powerHost, c.powerPort, "/status.xml", c.powerTimeout, c.powerUser, c.powerPass, body, err))
        return false;
    std::regex re("<relay" + std::to_string(c.motorRelay) + ">\\s*([01])\\s*</relay");
    std::smatch m;
    if (!std::regex_search(body, m, re))
    {
        err = "status.xml: motor relay not found";
        return false;
    }
    state = (m[1].str() == "1") ? 1 : 0;
    return true;
}

void RollOffRoof::setWorkerError(const std::string &e)
{
    std::lock_guard<std::mutex> lk(errM_);
    workerErr_ = e;
    workerErrNew_ = true;
}

void RollOffRoof::post(JobType t, int arg, bool front)
{
    Job j{t, arg, makeCfg()};
    if (t == J_POWER) pendingPower_++;
    if (t == J_STATUS) pendingStatus_++;
    {
        std::lock_guard<std::mutex> lk(qm_);
        if (front) q_.push_front(j);
        else q_.push_back(j);
    }
    qcv_.notify_one();
}

// Drop queued OPEN/CLOSE pulses (a pulse already in flight cannot be recalled).
void RollOffRoof::clearMotionJobs()
{
    std::lock_guard<std::mutex> lk(qm_);
    for (auto it = q_.begin(); it != q_.end();)
    {
        if (it->type == J_PULSE && it->arg != it->cfg.relayStop)
            it = q_.erase(it);
        else
            ++it;
    }
}

void RollOffRoof::workerLoop()
{
    std::unique_lock<std::mutex> lk(qm_);
    while (workerRun_)
    {
        if (q_.empty())
        {
            qcv_.wait(lk);
            continue;
        }
        Job j = q_.front();
        q_.pop_front();
        lk.unlock();
        runJob(j);
        lk.lock();
    }
}

void RollOffRoof::runJob(const Job &j)
{
    const Cfg &c = j.cfg;
    std::string body, err;
    char path[64];

    if (j.type == J_PULSE)
    {
        int code = 2 * (j.arg - 1);                 // relay OFF code; ON = +1
        int active = c.pulseRelayOn ? code + 1 : code;
        int idle = c.pulseRelayOn ? code : code + 1;

        snprintf(path, sizeof(path), "/%d/%02d", c.relayAddr, active);
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++)
            ok = httpGet(c.relayHost, c.relayPort, path, c.relayTimeout, "", "", body, err);
        if (!ok)
        {
            setWorkerError("Pulse on relay " + std::to_string(j.arg) + " failed: " + err);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(c.pulseMs));
        snprintf(path, sizeof(path), "/%d/%02d", c.relayAddr, idle);
        ok = false;
        for (int attempt = 0; attempt < 6 && !ok; attempt++)
        {
            ok = httpGet(c.relayHost, c.relayPort, path, c.relayTimeout, "", "", body, err);
            if (!ok)
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (!ok)
            setWorkerError("RELEASE of relay " + std::to_string(j.arg) + " failed (relay may be stuck on!): " + err);
    }
    else if (j.type == J_POWER)
    {
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++)
        {
            kmt::Style st = static_cast<kmt::Style>(kmStyle_.load());
            kmt::Style before = st;
            ok = kmt::setRelay(c.powerHost, c.powerPort, c.powerUser, c.powerPass, c.powerTimeout, c.motorRelay,
                               j.arg != 0, st, err);
            if (st != before)
            {
                kmStyle_ = static_cast<int>(st);
                styleSwitched_ = true;
            }
        }
        if (!ok)
            setWorkerError("Motor power command failed: " + err);
        else
        {
            int st = -1;
            if (readMotorRelay(c, st, err))
                powerState_ = st;
            else
                powerState_ = j.arg ? 1 : 0; // command accepted, readback failed
        }
        pendingPower_--;
    }
    else if (j.type == J_STATUS)
    {
        int st = -1;
        if (readMotorRelay(c, st, err))
            powerState_ = st;
        else
        {
            powerState_ = -1;
            setWorkerError("Motor power status: " + err);
        }
        pendingStatus_--;
    }
}

// =====================================================================================
//  Connect / disconnect
// =====================================================================================
bool RollOffRoof::Connect()
{
    Cfg c = makeCfg();
    std::string body, err;

    // Synchronous check of both boxes so that a wrong address is reported at connect time.
    char path[32];
    snprintf(path, sizeof(path), "/%d/99", c.relayAddr);
    if (!httpGet(c.relayHost, c.relayPort, path, c.relayTimeout, "", "", body, err))
    {
        LOGF_ERROR("16-port relay box (%s) not reachable: %s", c.relayHost.c_str(), err.c_str());
        return false;
    }
    int st = -1;
    if (!readMotorRelay(c, st, err))
    {
        LOGF_ERROR("Motor power box (%s) not usable: %s", c.powerHost.c_str(), err.c_str());
        return false;
    }
    powerState_ = st;

    motion_ = M_IDLE;
    mountKnown_ = imuKnown_ = false;
    allowed_ = false;
    lastReason_.clear();
    pendingPower_ = pendingStatus_ = 0;
    lastPowerCmdNs_ = lastStatusReqNs_ = nowNs();

    workerRun_ = true;
    worker_ = std::thread(&RollOffRoof::workerLoop, this);
    resnoop();

    LOGF_INFO("Connected. Relay box %s, power box %s (motor relay %d is %s).", c.relayHost.c_str(),
              c.powerHost.c_str(), c.motorRelay, st ? "ON" : "OFF");
    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool RollOffRoof::Disconnect()
{
    Cfg c = makeCfg();
    {
        std::lock_guard<std::mutex> lk(qm_);
        workerRun_ = false;
        q_.clear();
    }
    qcv_.notify_all();
    if (worker_.joinable())
        worker_.join();

    // Unsupervised roof: take the motor power away (a move in progress is stopped by this).
    if (policy(POL_CUT))
    {
        std::string err;
        kmt::Style st = static_cast<kmt::Style>(kmStyle_.load());
        if (kmt::setRelay(c.powerHost, c.powerPort, c.powerUser, c.powerPass, c.powerTimeout, c.motorRelay, false, st, err))
            LOG_INFO("Motor power switched off on disconnect.");
        else
            LOGF_ERROR("Could not switch motor power off on disconnect: %s", err.c_str());
    }
    if (motion_ != M_IDLE)
    {
        LOG_WARN("Disconnected while the roof was moving: its state is now unknown.");
        motion_ = M_IDLE;
        applyMark(MARK_UNKNOWN, true);
    }
    return true;
}

// =====================================================================================
//  Snooping
// =====================================================================================
void RollOffRoof::resnoop()
{
    IDSnoopDevice(SourcesTP[SRC_MOUNT].getText(), "TELESCOPE_PARK");
    IDSnoopDevice(SourcesTP[SRC_FOCUSER].getText(), "SCOPE_SAFETY");
}

bool RollOffRoof::ISSnoopDevice(XMLEle *root)
{
    const char *tag = tagXMLEle(root);
    const char *dev = findXMLAttValu(root, "device");
    const char *nm = findXMLAttValu(root, "name");
    const char *mountDev = SourcesTP[SRC_MOUNT].getText();
    const char *focDev = SourcesTP[SRC_FOCUSER].getText();

    if (strcmp(tag, "delProperty") == 0)
    {
        if (strcmp(dev, mountDev) == 0 && (nm[0] == '\0' || strcmp(nm, "TELESCOPE_PARK") == 0))
        {
            mountKnown_ = false;
            mountParked_ = false;
        }
        if (strcmp(dev, focDev) == 0 && (nm[0] == '\0' || strcmp(nm, "SCOPE_SAFETY") == 0))
        {
            imuKnown_ = false;
            imuSafe_ = false;
        }
    }
    else if (strcmp(dev, mountDev) == 0 && strcmp(nm, "TELESCOPE_PARK") == 0 &&
             (strcmp(tag, "defSwitchVector") == 0 || strcmp(tag, "setSwitchVector") == 0))
    {
        bool parkOn = false;
        for (XMLEle *ep = nextXMLEle(root, 1); ep != nullptr; ep = nextXMLEle(root, 0))
            if (strcmp(findXMLAttValu(ep, "name"), "PARK") == 0)
                parkOn = (strcmp(pcdataXMLEle(ep), "On") == 0);
        bool stateOk = (strcmp(findXMLAttValu(root, "state"), "Ok") == 0);
        mountKnown_ = true;
        mountParked_ = parkOn && stateOk;
    }
    else if (strcmp(dev, focDev) == 0 && strcmp(nm, "SCOPE_SAFETY") == 0 &&
             (strcmp(tag, "defLightVector") == 0 || strcmp(tag, "setLightVector") == 0))
    {
        for (XMLEle *ep = nextXMLEle(root, 1); ep != nullptr; ep = nextXMLEle(root, 0))
            if (strcmp(findXMLAttValu(ep, "name"), "SAFE") == 0)
            {
                imuKnown_ = true;
                imuSafe_ = (strcmp(pcdataXMLEle(ep), "Ok") == 0);
                imuSeenNs_ = nowNs();
            }
    }
    return INDI::Dome::ISSnoopDevice(root);
}

// =====================================================================================
//  Safety logic
// =====================================================================================
bool RollOffRoof::moveAllowed(std::string &reason) const
{
    if (policy(POL_OVERRIDE))
        return true;
    reason.clear();
    if (policy(POL_PARKED) && !(mountKnown_ && mountParked_))
        reason += mountKnown_ ? "mount is not parked; " : "no TELESCOPE_PARK from the mount device; ";
    if (policy(POL_IMU))
    {
        bool fresh = imuKnown_ && (nowNs() - imuSeenNs_) < kImuStaleNs;
        if (!fresh)
            reason += "no SCOPE_SAFETY data from the focuser driver; ";
        else if (!imuSafe_)
            reason += "scope is outside the IMU safe zone; ";
    }
    return reason.empty();
}

void RollOffRoof::evaluateSafety()
{
    std::string reason;
    bool allowed = moveAllowed(reason);
    bool changed = (allowed != lastAllowedPub_) || (reason != lastReason_);

    if (changed)
    {
        if (allowed)
            LOG_INFO(policy(POL_OVERRIDE) ? "Safety OVERRIDE active: movement allowed." : "Safe: movement allowed.");
        else
            LOGF_WARN("Movement NOT allowed: %s", reason.c_str());
    }
    allowed_ = allowed;
    lastReason_ = reason;

    int ps = powerState_.load();
    bool fresh = imuKnown_ && (nowNs() - imuSeenNs_) < kImuStaleNs;
    StatusLP[ST_PARKED].setState(mountKnown_ ? (mountParked_ ? IPS_OK : IPS_ALERT) : IPS_IDLE);
    StatusLP[ST_IMU].setState(fresh ? (imuSafe_ ? IPS_OK : IPS_ALERT) : IPS_IDLE);
    StatusLP[ST_POWER].setState(ps < 0 ? IPS_IDLE : (ps ? IPS_OK : IPS_BUSY));
    StatusLP[ST_ALLOWED].setState(allowed ? IPS_OK : IPS_ALERT);
    StatusLP.setState(allowed ? IPS_OK : IPS_ALERT);
    if (changed || (tickPub_++ % 10) == 0)
        StatusLP.apply();
    lastAllowedPub_ = allowed;
}

void RollOffRoof::emergencyStop(const char *why)
{
    LOGF_ERROR("EMERGENCY STOP: %s", why);
    clearMotionJobs();
    post(J_PULSE, static_cast<int>(RelayNP[R_STOP].getValue()), true);
    if (policy(POL_CUT))
        post(J_POWER, 0, true);   // runs first: cutting power stops the motor immediately
    motion_ = M_IDLE;
    setShutterState(SHUTTER_UNKNOWN);
    setDomeState(DOME_IDLE);
    applyMark(MARK_UNKNOWN, true);
}

// =====================================================================================
//  Roof commands
// =====================================================================================
IPState RollOffRoof::startMotion(bool open, bool viaPark)
{
    ShutterState cur = getShutterState();
    if (motion_ != M_IDLE)
    {
        if ((motion_ == M_OPENING) == open)
            return IPS_BUSY;
        LOG_ERROR("Roof is moving the other way: press STOP/abort first.");
        return IPS_ALERT;
    }
    if ((open && cur == SHUTTER_OPENED) || (!open && cur == SHUTTER_CLOSED))
    {
        LOGF_INFO("Roof is already %s.", open ? "open" : "closed");
        return IPS_OK;
    }

    std::string reason;
    if (!moveAllowed(reason))
    {
        LOGF_ERROR("Refusing to %s the roof: %s", open ? "open" : "close", reason.c_str());
        return IPS_ALERT;
    }

    if (policy(POL_CUT) && powerState_.load() != 1)
        post(J_POWER, 1);
    post(J_PULSE, static_cast<int>(RelayNP[open ? R_OPEN : R_CLOSE].getValue()));

    motion_ = open ? M_OPENING : M_CLOSING;
    viaPark_ = viaPark;
    double secs = TravelNP[open ? T_OPEN : T_CLOSE].getValue();
    motionEndNs_ = nowNs() + static_cast<int64_t>((secs + 1.0) * 1e9);
    setShutterState(SHUTTER_MOVING);
    setDomeState(viaPark ? (open ? DOME_UNPARKING : DOME_PARKING) : DOME_MOVING);
    applyMark(MARK_UNKNOWN, false);
    LOGF_INFO("%s roof (%.0f s travel assumed).", open ? "Opening" : "Closing", secs);
    return IPS_BUSY;
}

void RollOffRoof::finishMotion()
{
    bool open = (motion_ == M_OPENING);
    motion_ = M_IDLE;
    setShutterState(open ? SHUTTER_OPENED : SHUTTER_CLOSED);
    if (viaPark_ || !open)
        SetParked(!open);
    setDomeState(open ? DOME_IDLE : DOME_PARKED);
    applyMark(open ? MARK_OPEN : MARK_CLOSED, true);
    LOGF_INFO("Roof assumed %s (travel time elapsed).", open ? "open" : "closed");
}

IPState RollOffRoof::ControlShutter(ShutterOperation operation)
{
    return startMotion(operation == SHUTTER_OPEN, false);
}

IPState RollOffRoof::Park()
{
    return startMotion(false, true);
}

IPState RollOffRoof::UnPark()
{
    return startMotion(true, true);
}

bool RollOffRoof::Abort()
{
    // STOP is always allowed, whatever the safety state.
    clearMotionJobs();
    post(J_PULSE, static_cast<int>(RelayNP[R_STOP].getValue()), true);
    if (motion_ != M_IDLE)
    {
        motion_ = M_IDLE;
        setShutterState(SHUTTER_UNKNOWN);
        setDomeState(DOME_IDLE);
        applyMark(MARK_UNKNOWN, true);
        LOG_WARN("Roof stopped: position unknown (no feedback).");
    }
    else
        LOG_INFO("STOP pulse sent.");
    return true;
}

// Reflect / set the remembered roof state.
void RollOffRoof::applyMark(int idx, bool save)
{
    MarkSP.reset();
    MarkSP[idx].setState(ISS_ON);
    MarkSP.setState(IPS_OK);
    MarkSP.apply();
    if (idx == MARK_OPEN)
    {
        setShutterState(SHUTTER_OPENED);
        SetParked(false);
    }
    else if (idx == MARK_CLOSED)
    {
        setShutterState(SHUTTER_CLOSED);
        SetParked(true);
    }
    else if (motion_ == M_IDLE)
        setShutterState(SHUTTER_UNKNOWN);
    if (save)
        saveConfig(true, MarkSP.getName());
}

// =====================================================================================
//  Periodic work (100 ms)
// =====================================================================================
void RollOffRoof::TimerHit()
{
    if (!isConnected())
        return;
    int64_t now = nowNs();

    evaluateSafety();

    // errors from the worker thread
    {
        std::lock_guard<std::mutex> lk(errM_);
        if (workerErrNew_)
        {
            workerErrNew_ = false;
            LOGF_ERROR("%s", workerErr_.c_str());
        }
    }

    if (styleSwitched_.exchange(false))
    {
        LOG_WARN("The power box answers 404 to /FF0N01 commands: using /relays.cgi toggle commands instead "
                 "(set POWER_COMMAND_STYLE and save to make this permanent).");
        PowerStyleSP.reset();
        PowerStyleSP[kmStyle_.load()].setState(ISS_ON);
        PowerStyleSP.setState(IPS_OK);
        PowerStyleSP.apply();
    }

    // roof timers / safety while moving
    if (motion_ != M_IDLE)
    {
        if (!allowed_)
            emergencyStop(lastReason_.c_str());
        else if (now >= motionEndNs_)
            finishMotion();
    }

    // motor power follows the safety state
    if (isConnected() && policy(POL_CUT))
    {
        int ps = powerState_.load();
        int want = allowed_ ? 1 : 0;
        if (ps != want && pendingPower_ == 0 && (now - lastPowerCmdNs_) >= kPowerRetryNs)
        {
            lastPowerCmdNs_ = now;
            LOGF_INFO("Motor power -> %s", want ? "ON" : "OFF");
            post(J_POWER, want, !want); // cutting power jumps the queue
        }
    }
    if (pendingStatus_ == 0 && pendingPower_ == 0 && (now - lastStatusReqNs_) >= kStatusEveryNs)
    {
        lastStatusReqNs_ = now;
        post(J_STATUS, 0);
    }

    SetTimer(getCurrentPollingPeriod());
}

// =====================================================================================
//  Property handlers
// =====================================================================================
bool RollOffRoof::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
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

        if (MarkSP.isNameMatch(name))
        {
            if (motion_ != M_IDLE)
            {
                LOG_ERROR("Cannot change the roof state while it is moving.");
                MarkSP.setState(IPS_ALERT);
                MarkSP.apply();
                return true;
            }
            MarkSP.update(states, names, n);
            int idx = MarkSP.findOnSwitchIndex();
            applyMark(idx < 0 ? MARK_UNKNOWN : idx, true);
            LOGF_INFO("Roof state set manually: %s.", idx == MARK_OPEN ? "open" : (idx == MARK_CLOSED ? "closed" : "unknown"));
            return true;
        }
        if (handle(PolicySP))
        {
            if (policy(POL_OVERRIDE))
                LOG_WARN("OVERRIDE is ON: mount park state and IMU are ignored.");
            return true;
        }
        if (handle(PulsePolSP))
            return true;
        if (handle(PowerStyleSP))
        {
            kmStyle_ = std::max(0, PowerStyleSP.findOnSwitchIndex());
            return true;
        }
    }
    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

bool RollOffRoof::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
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
        if (handle(RelayNP) || handle(PowerNP) || handle(TravelNP))
            return true;
    }
    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}

bool RollOffRoof::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        auto handle = [&](INDI::PropertyText & p) -> bool
        {
            if (!p.isNameMatch(name))
                return false;
            bool ok = p.update(texts, names, n);
            p.setState(ok ? IPS_OK : IPS_ALERT);
            p.apply();
            return true;
        };
        if (handle(RelayHostTP) || handle(PowerHostTP))
            return true;
        if (handle(SourcesTP))
        {
            mountKnown_ = imuKnown_ = false;
            resnoop();
            return true;
        }
    }
    return INDI::Dome::ISNewText(dev, name, texts, names, n);
}
