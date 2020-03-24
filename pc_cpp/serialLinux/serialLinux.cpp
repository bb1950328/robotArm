// from https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

int serial_port = open("/dev/ttyUSB0", O_RDWR);

# Check for errors
if (serial_port < 0) {
    printf("Error %i from open: %s maybe try sudo adduser $USER dialout?\n", errno, strerror(errno));
}

// Create new termios struc, we call it 'tty' for convention
struct termios tty;
memset(&tty, 0, sizeof tty);

// Read in existing settings, and handle any error
if(tcgetattr(serial_port, &tty) != 0) {
printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
}

tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)

tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)

tty.c_cflag |= CS8; // 8 bits per byte (most common)

tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

tty.c_lflag &= ~ICANON; // non canonical mode

tty.c_lflag &= ~ECHO; // Disable echo
tty.c_lflag &= ~ECHOE; // Disable erasure
tty.c_lflag &= ~ECHONL; // Disable new-line echo

tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

void set_baudrate(int baud) {
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
}

write()

