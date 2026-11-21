/*
 * INDI driver: KMTronic 8-channel WEB LAN relay controller.
 *   GET /FF0N01 (relay N on), /FF0N00 (relay N off), GET /status.xml -> <relayN>0|1</relayN>
 *   HTTP basic authentication.
 * The INDI switches show POWER state (ON = load powered); normally-closed ports are inverted.
 */
#pragma once

#include <defaultdevice.h>
#include <string>
#include "../common/kmtronic_cmd.h"

class KMTronic8 : public INDI::DefaultDevice
{
    public:
        KMTronic8();
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
        static constexpr int N = 8;

        bool request(const std::string &path, std::string &body);
        bool refreshStatus();
        bool setPower(int idx, bool on);
        bool isNC(int idx) const;
        bool powerOf(int idx) const;
        void publishPorts(bool force);
        void applyNames();

        INDI::PropertyText   HostTP   {3};   // host, user, password
        INDI::PropertyNumber NetNP    {2};   // port, timeout
        INDI::PropertyText   NamesTP  {N};
        INDI::PropertySwitch WiringSP {N};
        INDI::PropertySwitch PortsSP  {N};
        INDI::PropertySwitch AllOffSP {1};
        INDI::PropertySwitch CmdStyleSP {3};   // auto / FF commands / relays.cgi toggle

        enum { H_HOST, H_USER, H_PASS };
        enum { NET_PORT, NET_TIMEOUT };

        kmt::Style style_ = kmt::AUTO;
        bool relayOn_[N] = {};
        bool lastPower_[N] = {};
        bool known_ = false;
        bool failing_ = false;
        bool configLoaded_ = false;
};
