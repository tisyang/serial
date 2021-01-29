#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

// simple serial port api, for win and linux
// author by TyK
// Github: https://github.com/tisyang/serial

#ifdef __cplusplus
#define SERIAL_API  extern "C"
#else
#define SERIAL_API
#endif // __cplusplus

#include <stddef.h>

#ifdef _WIN32
#include <Windows.h>

#define SERIAL  HANDLE
#define INVALID_SERIAL  INVALID_HANDLE_VALUE

#define BAUDRATE    int
#define BAUDRATE_LITERAL(n) (n)

#else
#include <termios.h>

#define SERIAL  int
#define INVALID_SERIAL  (-1)
#define BAUDRATE    speed_t
#define BAUDRATE_LITERAL(n) B##n

#endif

// general baudrates

#define SERIAL_B300       BAUDRATE_LITERAL(300)
#define SERIAL_B600       BAUDRATE_LITERAL(600)
#define SERIAL_B1200      BAUDRATE_LITERAL(1200)
#define SERIAL_B2400      BAUDRATE_LITERAL(2400)
#define SERIAL_B4800      BAUDRATE_LITERAL(4800)
#define SERIAL_B9600      BAUDRATE_LITERAL(9600)
#define SERIAL_B19200     BAUDRATE_LITERAL(19200)
#define SERIAL_B38400     BAUDRATE_LITERAL(38400)
#define SERIAL_B57600     BAUDRATE_LITERAL(57600)
#define SERIAL_B115200    BAUDRATE_LITERAL(115200)
#define SERIAL_B230400    BAUDRATE_LITERAL(230400)
#define SERIAL_B460800    BAUDRATE_LITERAL(460800)
#define SERIAL_B921600    BAUDRATE_LITERAL(921600)

// get BAUDRATE from number.
// return 0 means OK, return -1 means error(NOT VALID BAUDRATE NUMBER).
SERIAL_API  int     serial_num2baudrate(int num, BAUDRATE *br);

// get number from BAUDRATE
// return 0 means OK, return -1 means error(NOT VALID BAUDRATE).
SERIAL_API  int     serial_baudrate2num(BAUDRATE br, int *num);

// open a serial port by dev name and baudrate.
// return valid SERIAL if success, otherwise return INVALID_SERIAL.
// BAUDRATE should use SERIAL_B???? macros.
SERIAL_API  SERIAL  serial_open(const char *dev, BAUDRATE br);

// read from a SERIAL object, in non-blocking mode, return read count in bytes.
// return >0 means bytes count of read data, return 0 if no data for reading, return -1 means error.
SERIAL_API  int     serial_read(SERIAL sp, void *buf, size_t sz);

// write into a SERIAL object, in non-blocking mode, return write count in bytes.
// return >0 means bytes count of written data, return 0 means maybe blocking, return -1 means error.
SERIAL_API  int     serial_write(SERIAL sp, const void *data, size_t sz);

// clear SERIAL buffer, both IN and OUT.
// return 0 means ok, return -1 means error.
SERIAL_API  int     serial_clear(SERIAL sp);

// close a SERIAL object.
// return 0 means ok, return -1 means error.
SERIAL_API  int     serial_close(SERIAL sp);

#endif // SERIAL_PORT_H

