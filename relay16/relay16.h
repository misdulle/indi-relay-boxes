/*
 * INDI driver: 16-port HTTP relay box (4 pages of 4 relays).
 *   http://IP/ADDR/<2-digit code>   relay n OFF = 2(n-1), ON = 2(n-1)+1
 *   http://IP/ADDR/99               status: 16 chars of 0/1
 * The INDI switch always shows POWER state (ON = load powered). Ports wired normally
 * closed (NC) have the relay command / state inverted.
 */
#pragma once

#include <defaultdevice.h>
#include <string>

class Relay16Box : public INDI::DefaultDevice
{
    public:
        Relay16Box();
        const char *getDefaultName() override;

        bool initProperties() override;
        void ISGetProperties(const char *dev) override;
        bool updateProperties() override;
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

    protected:
        bool Connect() override;
        bool Disconnect() override;
        void TimerHit() override;
        bool saveConfigItems(FILE *fp) override;

    private:
        static constexpr int N = 16;

        bool request(int code, std::string &body);   // GET /ADDR/<code>
        bool refreshStatus();                          // GET /ADDR/99, updates relayOn_
        bool setPower(int idx, bool on);               // idx 0..15
        void parsePage(const std::string &body);       // "Relay-NN: ON/OFF" entries
        bool isNC(int idx) const;
        bool powerOf(int idx) const;
        void publishPorts(bool force);
        void applyNames();

        INDI::PropertyText   HostTP   {1};
        INDI::PropertyNumber NetNP    {3};
        INDI::PropertyText   NamesTP  {N};
        INDI::PropertySwitch WiringSP {N};
        INDI::PropertySwitch PortsSP  {N};
        INDI::PropertySwitch AllOffSP {1};

        enum { NET_PORT, NET_ADDR, NET_TIMEOUT };

        bool relayOn_[N] = {};     // raw relay state as reported by the box
        bool lastPower_[N] = {};   // last published power state
        bool known_ = false;
        bool failing_ = false;
        bool configLoaded_ = false;
};
