#include "kmtronic8.h"
#include "../common/http_client.h"

#include <indilogger.h>
#include <cstdio>
#include <cstring>
#include <memory>
#include <regex>
#include <algorithm>

static std::unique_ptr<KMTronic8> device(new KMTronic8());

KMTronic8::KMTronic8()
{
    setVersion(1, 0);
    setDefaultPollingPeriod(2000);
}

const char *KMTronic8::getDefaultName()
{
    return "KMTronic 8 Relay LAN";
}

bool KMTronic8::initProperties()
{
    INDI::DefaultDevice::initProperties();
    const char *dev = getDeviceName();

    // Flag this device as Auxiliary in DRIVER_INFO. Done through the property because
    // DefaultDevice::setDriverInterface() is not linkable on every INDI 2.x build.
    {
        INDI::PropertyText info = getText("DRIVER_INFO");
        if (info.isValid())
        {
            auto *w = info.findWidgetByName("DRIVER_INTERFACE");
            if (w != nullptr)
            {
                char b[16];
                snprintf(b, sizeof(b), "%u", static_cast<unsigned>(AUX_INTERFACE));
                w->setText(b);
            }
        }
    }

    HostTP[H_HOST].fill("HOST", "IP / host", "192.168.1.23");
    HostTP[H_USER].fill("USER", "User (empty = no auth)", "admin");
    HostTP[H_PASS].fill("PASSWORD", "Password", "admin");
    HostTP.fill(dev, "BOX_HOST", "Relay box", "Setup", IP_RW, 0, IPS_IDLE);

    NetNP[NET_PORT].fill("TCP_PORT", "TCP port", "%.0f", 1, 65535, 1, 80);
    NetNP[NET_TIMEOUT].fill("TIMEOUT_MS", "HTTP timeout (ms)", "%.0f", 200, 30000, 100, 2000);
    NetNP.fill(dev, "BOX_NETWORK", "Network", "Setup", IP_RW, 0, IPS_IDLE);

    for (int i = 0; i < N; i++)
    {
        char nm[16], lb[32], dflt[16];
        snprintf(nm, sizeof(nm), "NAME%02d", i + 1);
        snprintf(lb, sizeof(lb), "Port %02d", i + 1);
        snprintf(dflt, sizeof(dflt), "Port %02d", i + 1);
        NamesTP[i].fill(nm, lb, dflt);
    }
    NamesTP.fill(dev, "PORT_NAMES", "Port names", "Setup", IP_RW, 0, IPS_IDLE);

    for (int i = 0; i < N; i++)
    {
        char nm[16], lb[48];
        snprintf(nm, sizeof(nm), "NC%02d", i + 1);
        snprintf(lb, sizeof(lb), "Port %02d normally closed", i + 1);
        WiringSP[i].fill(nm, lb, ISS_OFF);
    }
    WiringSP.fill(dev, "PORT_WIRING", "Wiring (off = normally open)", "Setup", IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    for (int i = 0; i < N; i++)
    {
        char nm[16];
        snprintf(nm, sizeof(nm), "PORT%02d", i + 1);
        PortsSP[i].fill(nm, NamesTP[i].getText(), ISS_OFF);
    }
    PortsSP.fill(dev, "PORTS", "Ports (ON = powered)", "Main Control", IP_RW, ISR_NOFMANY, 0, IPS_IDLE);

    CmdStyleSP[0].fill("STYLE_AUTO", "Auto (FF, fall back to toggle)", ISS_ON);
    CmdStyleSP[1].fill("STYLE_FF", "/FF0N01 on, /FF0N00 off", ISS_OFF);
    CmdStyleSP[2].fill("STYLE_CGI", "/relays.cgi?relay=N (toggle)", ISS_OFF);
    CmdStyleSP.fill(dev, "COMMAND_STYLE", "Command style", "Setup", IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    AllOffSP[0].fill("ALL_OFF", "All off", ISS_OFF);
    AllOffSP.fill(dev, "ALL_OFF_CMD", "All loads", "Main Control", IP_RW, ISR_ATMOST1, 0, IPS_IDLE);
    return true;
}

void KMTronic8::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    defineProperty(HostTP);
    defineProperty(NetNP);
    defineProperty(NamesTP);
    defineProperty(WiringSP);
    defineProperty(CmdStyleSP);

    if (!configLoaded_)
    {
        configLoaded_ = true;
        loadConfig(true, HostTP.getName());
        loadConfig(true, NetNP.getName());
        loadConfig(true, NamesTP.getName());
        loadConfig(true, WiringSP.getName());
        loadConfig(true, CmdStyleSP.getName());
        style_ = static_cast<kmt::Style>(std::max(0, CmdStyleSP.findOnSwitchIndex()));
        applyNames();
    }
}

bool KMTronic8::updateProperties()
{
    INDI::DefaultDevice::updateProperties();
    if (isConnected())
    {
        applyNames();
        publishPorts(true);
        defineProperty(PortsSP);
        defineProperty(AllOffSP);
    }
    else
    {
        deleteProperty(PortsSP.getName());
        deleteProperty(AllOffSP.getName());
    }
    return true;
}

bool KMTronic8::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    HostTP.save(fp);   // note: the password is stored in clear text in the config file
    NetNP.save(fp);
    NamesTP.save(fp);
    WiringSP.save(fp);
    CmdStyleSP.save(fp);
    return true;       // PORTS is deliberately not saved
}

void KMTronic8::applyNames()
{
    for (int i = 0; i < N; i++)
        PortsSP[i].setLabel(NamesTP[i].getText());
}

bool KMTronic8::isNC(int idx) const { return WiringSP[idx].getState() == ISS_ON; }
bool KMTronic8::powerOf(int idx) const { return isNC(idx) ? !relayOn_[idx] : relayOn_[idx]; }

void KMTronic8::publishPorts(bool force)
{
    bool changed = force;
    for (int i = 0; i < N; i++)
    {
        bool p = known_ && powerOf(i);
        if (p != lastPower_[i])
            changed = true;
        lastPower_[i] = p;
        PortsSP[i].setState(p ? ISS_ON : ISS_OFF);
    }
    IPState st = !known_ ? IPS_IDLE : (failing_ ? IPS_ALERT : IPS_OK);
    if (PortsSP.getState() != st)
        changed = true;
    PortsSP.setState(st);
    if (changed)
        PortsSP.apply();
}

bool KMTronic8::request(const std::string &path, std::string &body)
{
    int status = 0;
    std::string err;
    std::string hdr = httpc::basicAuthHeader(HostTP[H_USER].getText(), HostTP[H_PASS].getText());
    if (!httpc::get(HostTP[H_HOST].getText(), static_cast<int>(NetNP[NET_PORT].getValue()), path,
                    static_cast<int>(NetNP[NET_TIMEOUT].getValue()), status, body, err, hdr))
    {
        if (!failing_)
            LOGF_ERROR("Request %s failed: %s", path.c_str(), err.c_str());
        return false;
    }
    if (status == 401)
    {
        if (!failing_)
            LOG_ERROR("Authentication failed (HTTP 401): check user and password.");
        return false;
    }
    if (status != 200)
    {
        if (!failing_)
            LOGF_ERROR("Request %s: HTTP status %d", path.c_str(), status);
        return false;
    }
    return true;
}

// <response><relay0>1</relay0><relay1>0</relay1> ... <relay8>0</relay8></response>
// Only relay1..relay8 are used; the unit also reports a stray relay0.
bool KMTronic8::refreshStatus()
{
    std::string body;
    bool ok = request("/status.xml", body);
    if (ok)
    {
        static const std::regex re("<relay([1-8])>\\s*([01])\\s*</relay");
        bool seen[N] = {};
        bool val[N] = {};
        int count = 0;
        for (std::sregex_iterator it(body.begin(), body.end(), re), end; it != end; ++it)
        {
            int i = std::stoi((*it)[1].str()) - 1;
            if (!seen[i]) { seen[i] = true; count++; }
            val[i] = ((*it)[2].str() == "1");
        }
        if (count == N)
        {
            for (int i = 0; i < N; i++)
                relayOn_[i] = val[i];
            known_ = true;
        }
        else
        {
            if (!failing_)
                LOG_ERROR("status.xml did not contain relay1..relay8.");
            LOGF_DEBUG("status.xml body: %s", body.c_str());
            ok = false;
        }
    }

    if (!ok)
    {
        failing_ = true;
        PortsSP.setState(IPS_ALERT);
        PortsSP.apply();
        return false;
    }
    if (failing_)
        LOG_INFO("Communication with the relay box restored.");
    failing_ = false;
    publishPorts(false);
    return true;
}

bool KMTronic8::setPower(int idx, bool on)
{
    bool relayWanted = isNC(idx) ? !on : on;
    kmt::Style before = style_;
    std::string err, path;
    if (!kmt::setRelay(HostTP[H_HOST].getText(), static_cast<int>(NetNP[NET_PORT].getValue()),
                       HostTP[H_USER].getText(), HostTP[H_PASS].getText(),
                       static_cast<int>(NetNP[NET_TIMEOUT].getValue()), idx + 1, relayWanted, style_, err, &path))
    {
        LOGF_ERROR("Switching relay %02d failed (%s): %s", idx + 1, path.c_str(), err.c_str());
        return false;
    }
    if (before == kmt::AUTO && style_ == kmt::CGI)
    {
        LOG_WARN("The box answers 404 to /FF0N01 commands: using /relays.cgi toggle commands instead "
                 "(set COMMAND_STYLE and save to make this permanent).");
        CmdStyleSP.reset();
        CmdStyleSP[2].setState(ISS_ON);
        CmdStyleSP.setState(IPS_OK);
        CmdStyleSP.apply();
    }
    LOGF_INFO("%s -> %s (relay %02d %s, %s)", NamesTP[idx].getText(), on ? "ON" : "OFF", idx + 1,
              relayWanted ? "ON" : "OFF", path.c_str());
    return true;
}

bool KMTronic8::Connect()
{
    failing_ = false;
    known_ = false;
    if (!refreshStatus())
    {
        LOG_ERROR("Cannot read the relay box status; check IP, port, user and password.");
        return false;
    }
    LOGF_INFO("Connected to %s:%.0f.", HostTP[H_HOST].getText(), NetNP[NET_PORT].getValue());
    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool KMTronic8::Disconnect()
{
    known_ = false; // relays are left as they are
    return true;
}

void KMTronic8::TimerHit()
{
    if (!isConnected())
        return;
    refreshStatus();
    SetTimer(getCurrentPollingPeriod());
}

bool KMTronic8::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (PortsSP.isNameMatch(name))
        {
            bool ok = true;
            for (int k = 0; k < n; k++)
            {
                for (int i = 0; i < N; i++)
                {
                    if (strcmp(names[k], PortsSP[i].getName()) != 0)
                        continue;
                    bool want = (states[k] == ISS_ON);
                    if (known_ && want == powerOf(i))
                        break;
                    ok = setPower(i, want) && ok;
                    break;
                }
            }
            bool synced = refreshStatus(); // authoritative state, also corrects the client display
            PortsSP.setState((ok && synced) ? IPS_OK : IPS_ALERT);
            PortsSP.apply();
            return true;
        }

        if (CmdStyleSP.isNameMatch(name))
        {
            CmdStyleSP.update(states, names, n);
            style_ = static_cast<kmt::Style>(std::max(0, CmdStyleSP.findOnSwitchIndex()));
            CmdStyleSP.setState(IPS_OK);
            CmdStyleSP.apply();
            return true;
        }

        if (WiringSP.isNameMatch(name))
        {
            WiringSP.update(states, names, n);
            WiringSP.setState(IPS_OK);
            WiringSP.apply();
            if (isConnected())
            {
                LOG_INFO("Wiring changed: displayed power states are re-derived from the relay states.");
                publishPorts(true);
            }
            return true;
        }

        if (AllOffSP.isNameMatch(name))
        {
            bool ok = refreshStatus();
            for (int i = 0; ok && i < N; i++)
                if (powerOf(i))
                    ok = setPower(i, false);
            refreshStatus();
            AllOffSP[0].setState(ISS_OFF);
            AllOffSP.setState(ok ? IPS_OK : IPS_ALERT);
            AllOffSP.apply();
            publishPorts(true);
            return true;
        }
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool KMTronic8::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && NetNP.isNameMatch(name))
    {
        bool ok = NetNP.update(values, names, n);
        NetNP.setState(ok ? IPS_OK : IPS_ALERT);
        NetNP.apply();
        return true;
    }
    return INDI::DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

bool KMTronic8::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (HostTP.isNameMatch(name))
        {
            bool ok = HostTP.update(texts, names, n);
            HostTP.setState(ok ? IPS_OK : IPS_ALERT);
            HostTP.apply();
            return true;
        }
        if (NamesTP.isNameMatch(name))
        {
            bool ok = NamesTP.update(texts, names, n);
            NamesTP.setState(ok ? IPS_OK : IPS_ALERT);
            NamesTP.apply();
            applyNames();
            if (isConnected())
            {
                deleteProperty(PortsSP.getName());
                publishPorts(true);
                defineProperty(PortsSP);
            }
            return true;
        }
    }
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}
