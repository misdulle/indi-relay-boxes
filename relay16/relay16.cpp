#include "relay16.h"
#include "../common/http_client.h"

#include <indilogger.h>
#include <cstdio>
#include <cstring>
#include <memory>
#include <regex>

static std::unique_ptr<Relay16Box> device(new Relay16Box());

Relay16Box::Relay16Box()
{
    setVersion(1, 0);
    setDefaultPollingPeriod(2000);
}

const char *Relay16Box::getDefaultName()
{
    return "Relay16";
}

bool Relay16Box::initProperties()
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

    HostTP[0].fill("HOST", "IP / host", "192.168.1.28");
    HostTP.fill(dev, "BOX_HOST", "Relay box", "Setup", IP_RW, 0, IPS_IDLE);

    NetNP[NET_PORT].fill("TCP_PORT", "TCP port", "%.0f", 1, 65535, 1, 80);
    NetNP[NET_ADDR].fill("ADDR", "Base address (ADDR)", "%.0f", 0, 99, 1, 30);
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

    AllOffSP[0].fill("ALL_OFF", "All off", ISS_OFF);
    AllOffSP.fill(dev, "ALL_OFF_CMD", "All loads", "Main Control", IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    return true;
}

void Relay16Box::ISGetProperties(const char *dev)
{
    INDI::DefaultDevice::ISGetProperties(dev);
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    defineProperty(HostTP);
    defineProperty(NetNP);
    defineProperty(NamesTP);
    defineProperty(WiringSP);

    if (!configLoaded_)
    {
        configLoaded_ = true;
        loadConfig(true, HostTP.getName());
        loadConfig(true, NetNP.getName());
        loadConfig(true, NamesTP.getName());
        loadConfig(true, WiringSP.getName());
        applyNames();
    }
}

bool Relay16Box::updateProperties()
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

bool Relay16Box::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    HostTP.save(fp);
    NetNP.save(fp);
    NamesTP.save(fp);
    WiringSP.save(fp);
    return true; // PORTS is deliberately not saved: connecting never switches anything
}

void Relay16Box::applyNames()
{
    for (int i = 0; i < N; i++)
        PortsSP[i].setLabel(NamesTP[i].getText());
}

// ---------------------------------------------------------------------------------------
bool Relay16Box::isNC(int idx) const
{
    return WiringSP[idx].getState() == ISS_ON;
}

// Power state of the load (what the user cares about), derived from the relay state.
bool Relay16Box::powerOf(int idx) const
{
    return isNC(idx) ? !relayOn_[idx] : relayOn_[idx];
}

void Relay16Box::publishPorts(bool force)
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

// ---------------------------------------------------------------------------------------
bool Relay16Box::request(int code, std::string &body)
{
    char path[48];
    snprintf(path, sizeof(path), "/%d/%02d", static_cast<int>(NetNP[NET_ADDR].getValue()), code);
    int status = 0;
    std::string err;
    if (!httpc::get(HostTP[0].getText(), static_cast<int>(NetNP[NET_PORT].getValue()), path,
                    static_cast<int>(NetNP[NET_TIMEOUT].getValue()), status, body, err))
    {
        if (!failing_)
            LOGF_ERROR("Request %s failed: %s", path, err.c_str());
        return false;
    }
    if (status != 200)
    {
        if (!failing_)
            LOGF_ERROR("Request %s: HTTP status %d", path, status);
        return false;
    }
    return true;
}

// Reply to any control command is the page of the 4 relays it belongs to:
//   Relay-01: <font color="#FF0000"> OFF&nbsp&nbsp</font>
void Relay16Box::parsePage(const std::string &body)
{
    static const std::regex re("Relay-(\\d\\d):\\s*<font[^>]*>\\s*(ON|OFF)");
    for (std::sregex_iterator it(body.begin(), body.end(), re), end; it != end; ++it)
    {
        int n = std::stoi((*it)[1].str());
        if (n >= 1 && n <= N)
            relayOn_[n - 1] = ((*it)[2].str() == "ON");
    }
}

// Status page: <a href=".../30/0000000000000000">0000000000000000</a> (relay 1 first)
bool Relay16Box::refreshStatus()
{
    std::string body;
    bool ok = request(99, body);
    if (ok)
    {
        static const std::regex re(">\\s*([01]{16})\\s*<");
        std::smatch m;
        if (std::regex_search(body, m, re))
        {
            std::string s = m[1].str();
            for (int i = 0; i < N; i++)
                relayOn_[i] = (s[i] == '1');
            known_ = true;
        }
        else
        {
            if (!failing_)
                LOG_ERROR("Status reply has no 16-digit state field.");
            LOGF_DEBUG("Status body: %s", body.c_str());
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

bool Relay16Box::setPower(int idx, bool on)
{
    bool relayWanted = isNC(idx) ? !on : on;
    int code = 2 * idx + (relayWanted ? 1 : 0);
    std::string body;
    if (!request(code, body))
        return false;
    parsePage(body);
    LOGF_INFO("%s -> %s (relay %02d %s, command %02d)", NamesTP[idx].getText(), on ? "ON" : "OFF", idx + 1,
              relayWanted ? "ON" : "OFF", code);
    return true;
}

// ---------------------------------------------------------------------------------------
bool Relay16Box::Connect()
{
    failing_ = false;
    known_ = false;
    if (!refreshStatus())
    {
        LOG_ERROR("Cannot read the relay box status; check IP, port and ADDR.");
        return false;
    }
    LOGF_INFO("Connected to %s:%.0f, ADDR %.0f.", HostTP[0].getText(), NetNP[NET_PORT].getValue(),
              NetNP[NET_ADDR].getValue());
    SetTimer(getCurrentPollingPeriod());
    return true;
}

bool Relay16Box::Disconnect()
{
    known_ = false; // relays are left exactly as they are
    return true;
}

void Relay16Box::TimerHit()
{
    if (!isConnected())
        return;
    refreshStatus();
    SetTimer(getCurrentPollingPeriod());
}

// ---------------------------------------------------------------------------------------
bool Relay16Box::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
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

bool Relay16Box::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
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

bool Relay16Box::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
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
                // Labels only reach the client when the property is (re)defined.
                deleteProperty(PortsSP.getName());
                publishPorts(true);
                defineProperty(PortsSP);
            }
            return true;
        }
    }
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}
