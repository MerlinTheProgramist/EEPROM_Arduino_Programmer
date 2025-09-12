#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <string>
#include <fstream>
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>

#include <boost/program_options.hpp>

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

void check_path_exists(const std::string& path){
  if(!std::filesystem::exists(path)){
    std::cerr << "path "<< std::quoted(path) << " does not exist!";
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv){
  namespace po = boost::program_options;

  po::options_description desc("");
  desc.add_options()
    ("help,h", "produces help message")
    ("input-file", po::value<std::string>()->required()->notifier(check_path_exists), "input file")
    ("port,p", po::value<std::string>()->required()->notifier(check_path_exists), "EEprogm programator port")
    ("baudrate,b", po::value<unsigned int>()->default_value(9600), "baudrate for the port");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  if(vm.count("help") || !vm.count("port")){
    std::cout << desc << '\n';
    // std::
    return EXIT_SUCCESS;
  }

  std::vector<uint8_t> data{};
  if(vm.count("input-file")){
    std::string filename = vm["intput-file"].as<std::string>();

    std::ifstream file(filename, std::ios::binary);

    // stop eating new-lines in binary mode
    file.unsetf(std::ios::skipws);

    // get it's size
    file.seekg(0, std::ios::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // reserve and read file
    data.reserve(file_size);
    data.insert(data.begin(),
                std::istream_iterator<uint8_t>(file),
                std::istream_iterator<uint8_t>());    
  }else{
    if(isatty(STDOUT_FILENO)){ // if pipe is used
      fprintf(stderr, "no input-file provided (or data piped)!");
      return EXIT_FAILURE;
    }
    // read data from stream until end
    std::for_each(std::istreambuf_iterator<char>(std::cin),
                  std::istreambuf_iterator<char>(),
                  [&data](const uint8_t c){
                      data.push_back(c);
                  });
  }


  std::string serial_path{vm["port"].as<std::string>()};
  int serial = open(serial_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if(serial < 0){
    std::cerr << "Failed when opening serial port:\n";
    std::cerr << '\t' << strerror(errno) << std::endl;
    return EXIT_FAILURE;
  }

  struct termios tty{};

  if (tcgetattr(serial, &tty) != 0)
  {
      std::cerr << "tcgetattr(" << serial_path << "): " << strerror(errno) << std::endl;
      return EXIT_FAILURE;
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

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  speed_t baudrate = get_baud(vm["baudrate"].as<int>());
  if(baudrate == -1){
    std::cerr << "Invalid baudrate!" << std::endl;
    return EXIT_FAILURE;
  }

  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  if (tcsetattr(serial, TCSANOW, &tty) != 0) {
      std::cerr << "tcgetattr("<< serial_path << "): " << strerror(errno) << std::endl;
      return EXIT_FAILURE;
  }

  const size_t CHUNK_SIZE = 4; // 32 bits (half of arduino's serial buffer)
  
  for(int index=0; index < data.size(); index+=CHUNK_SIZE){
    // write header

    // write data
    write(serial, &data[index], CHUNK_SIZE);

    uint8_t recv[2];
    read(serial, recv, 2)
  }
}
