#include "serial.h"
#include <stdio.h>

#ifndef _WIN32
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#endif

struct BaudratePair {
    int      number;
    BAUDRATE br;
};

#define BAUDRATE_PAIR(n)    { n, BAUDRATE_LITERAL(n) }

static const struct BaudratePair BAUDRATEPAIRS[] = {
    BAUDRATE_PAIR(300),
    BAUDRATE_PAIR(600),
    BAUDRATE_PAIR(1200),
    BAUDRATE_PAIR(2400),
    BAUDRATE_PAIR(4800),
    BAUDRATE_PAIR(9600),
    BAUDRATE_PAIR(19200),
    BAUDRATE_PAIR(38400),
    BAUDRATE_PAIR(57600),
    BAUDRATE_PAIR(115200),
    BAUDRATE_PAIR(230400),
    BAUDRATE_PAIR(460800),
    BAUDRATE_PAIR(0),
};

int     serial_num2baudrate(int num, BAUDRATE *br)
{
    const struct BaudratePair *pair = BAUDRATEPAIRS;
    for (; pair->number; pair++) {
        if (num == pair->number) {
            *br = pair->br;
            return 0;
        }
    }
    return -1;
}

int     serial_baudrate2num(BAUDRATE br, int *num)
{
    const struct BaudratePair *pair = BAUDRATEPAIRS;
    for (; pair->number; pair++) {
        if (br == pair->br) {
            *num = pair->number;
            return 0;
        }
    }
    return -1;
}

SERIAL  serial_open(const char *dev, BAUDRATE br)
{
    SERIAL serial = INVALID_SERIAL;
#ifdef _WIN32
    COMMCONFIG cc = {0};
    DWORD sz = sizeof(cc);
    COMMTIMEOUTS co = { MAXDWORD, 0, 0, 0, 0};
    char dcb[64] = "";
    char name[128] = "";

    snprintf(name, sizeof(name), "\\\\.\\%s", dev);
    serial = CreateFile(name,
                        GENERIC_READ|GENERIC_WRITE,
                        0,
                        0,
                        OPEN_EXISTING,
                        0,
                        NULL);
    if (serial == INVALID_HANDLE_VALUE) {
        return INVALID_SERIAL;
    }
    if (!GetCommConfig(serial, &cc, &sz)) {
        CloseHandle(serial);
        return INVALID_SERIAL;
    }
    snprintf(dcb, sizeof(dcb),
             "baud=%d parity=%c data=%d stop=%d",
             br, 'N', 8, 1);
    if (!BuildCommDCB(dcb, &cc.dcb)) {
        CloseHandle(serial);
        return INVALID_SERIAL;
    }
    SetCommConfig(serial, &cc, sz);
    SetCommTimeouts(serial, &co);
    DWORD error;
    ClearCommError(serial, &error, NULL);
    PurgeComm(serial, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);
    return serial;
#else
    struct termios ios = {0};
    if ((serial = open(dev, O_RDWR|O_NOCTTY|O_NONBLOCK)) == -1) {
        return INVALID_SERIAL;
    }
    tcgetattr(serial, &ios);
    ios.c_iflag = 0;
    ios.c_oflag = 0;
    ios.c_lflag = 0;
    ios.c_cc[VMIN] = 0;
    ios.c_cc[VTIME] = 0;
    cfsetispeed(&ios, br);
    cfsetospeed(&ios, br);
    // data bit 8
    ios.c_cflag &= ~CSIZE;
    ios.c_cflag |= CS8;
    // ignore modem controls
    ios.c_cflag |= (CLOCAL | CREAD);
    // no parity bit
    ios.c_cflag &= ~(PARENB | PARODD);
    // stop bit 1
    ios.c_cflag &= ~CSTOPB;
    // no harware flowcontrol
    ios.c_cflag &= ~CRTSCTS;
    tcsetattr(serial, TCSANOW, &ios);
    tcflush(serial, TCIOFLUSH);
    return serial;
#endif
}

int serial_read(SERIAL sp, void *buf, size_t sz)
{
#ifdef _WIN32
    DWORD rc;
    if (!ReadFile(sp, buf, sz, &rc, NULL)) {
        return -1;
    } else {
        return rc;
    }
#else
    int rv = read(sp, buf, sz);
    if (rv == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    } else {
        return rv;
    }
#endif
}

int serial_write(SERIAL sp, const void *data, size_t sz)
{
#ifdef _WIN32
    DWORD wc;
    if (!WriteFile(sp, data, sz, &wc, NULL)) {
        return -1;
    } else {
        return wc;
    }
#else
    int rv = write(sp, data, sz);
    if (rv == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    } else {
        return rv;
    }
#endif
}

int serial_clear(SERIAL sp)
{
#ifdef _WIN32
    DWORD error;
    if (!ClearCommError(sp, &error, NULL)) {
        return -1;
    }
    if (!PurgeComm(sp, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR)) {
        return -1;
    }
    return 0;
#else
    int rv = tcflush(sp, TCIOFLUSH);
    return rv;
#endif
}

int serial_close(SERIAL sp)
{
#ifdef _WIN32
    if (!CloseHandle(sp)) {
        return -1;
    }
    return 0;
#else
    int rv = close(sp);
    return rv;
#endif
}



