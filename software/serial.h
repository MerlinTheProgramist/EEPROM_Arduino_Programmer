#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
inline 
speed_t get_baud(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return -1;
    }
}

inline
int setup_serial(std::string_view serial_path, int baudrate){
  int serial = open(serial_path.data(), O_RDWR | O_NOCTTY | O_SYNC /* | O_NDELAY */);

  if(serial < 0){
    std::cerr << "Failed when opening serial port:\n";
    std::cerr << '\t' << strerror(errno) << std::endl;
    return -1;
  }
  struct termios tty{};

  if (tcgetattr(serial, &tty) != 0)
  {
      std::cerr << "tcgetattr(" << serial_path << "): " << strerror(errno) << std::endl;
      return -1;
  }
  

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10; // 10 ds = 1s
  tty.c_cc[VMIN] = 0; // minimum number of characters to read

  speed_t baudrate_s = get_baud(baudrate);
  if(baudrate_s == -1){
    std::cerr << "Invalid baudrate!" << std::endl;
    return -1;
  }

  cfsetispeed(&tty, baudrate_s);
  cfsetospeed(&tty, baudrate_s);

  if (tcsetattr(serial, TCSANOW, &tty) != 0) {
      std::cerr << "tcsetattr("<< serial_path << "): " << strerror(errno) << std::endl;
      return -1;
  }
  return serial;
}


