#pragma once
// Minimal GPIO output helper on top of the kernel character-device uAPI v2
// (linux/gpio.h, kernel >= 5.10). No libgpiod needed, works on Pi 5 (RP1).
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <string>

class GpioOutputs
{
    public:
        ~GpioOutputs() { release(); }

        // Request lines as outputs, all initially driven to initial[i] (0/1, physical level).
        bool request(const std::string &chipPath, const unsigned *offsets, const int *initial,
                     unsigned n, const char *consumer, std::string &err)
        {
            release();
            if (n == 0 || n > GPIO_V2_LINES_MAX) { err = "bad line count"; return false; }
            int chipFd = open(chipPath.c_str(), O_RDWR | O_CLOEXEC);
            if (chipFd < 0) { err = chipPath + ": " + strerror(errno); return false; }

            gpio_v2_line_request req;
            memset(&req, 0, sizeof(req));
            for (unsigned i = 0; i < n; i++)
                req.offsets[i] = offsets[i];
            req.num_lines = n;
            strncpy(req.consumer, consumer, sizeof(req.consumer) - 1);
            req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
            req.config.num_attrs = 1;
            req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
            uint64_t bits = 0;
            for (unsigned i = 0; i < n; i++)
                if (initial[i]) bits |= (1ULL << i);
            req.config.attrs[0].attr.values = bits;
            req.config.attrs[0].mask = (n == 64) ? ~0ULL : ((1ULL << n) - 1);

            int rc = ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &req);
            int e = errno;
            close(chipFd);
            if (rc < 0 || req.fd < 0) { err = std::string("GPIO_V2_GET_LINE_IOCTL: ") + strerror(e); return false; }
            fd_ = req.fd;
            n_ = n;
            return true;
        }

        // Set a single line (index within the request), physical level 0/1.
        bool set(unsigned idx, int level)
        {
            if (fd_ < 0 || idx >= n_) return false;
            gpio_v2_line_values v;
            v.bits = level ? (1ULL << idx) : 0;
            v.mask = (1ULL << idx);
            return ioctl(fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) == 0;
        }

        void release()
        {
            if (fd_ >= 0) { close(fd_); fd_ = -1; }
        }
        bool isOpen() const { return fd_ >= 0; }

    private:
        int fd_ = -1;
        unsigned n_ = 0;
};
