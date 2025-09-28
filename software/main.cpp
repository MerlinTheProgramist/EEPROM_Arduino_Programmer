#include <boost/program_options/cmdline.hpp>
#include <boost/program_options/positional_options.hpp>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <fstream>
#include <unistd.h>

#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <boost/program_options.hpp>
#include <variant>

#include "../shared/commands.h"
#include "../shared/config.h"
#include "serial.h"

void check_path_exists(const std::string& path){
  if(!std::filesystem::exists(path)){
    std::cerr << "path "<< std::quoted(path) << " does not exist!";
    exit(EXIT_FAILURE);
  }
}

/// @param timeout in ms
int wait_for_read(int fd, uint8_t buffer[], size_t count, int timeout){
  {
    struct termios tm;
    tcgetattr(fd, &tm);
    tm.c_cc[VTIME] = 1; // inter-character timeout 100ms, valid after first char recvd
    tm.c_cc[VMIN] = count; // block until n characters are read
    tcsetattr(fd, TCSANOW, &tm);
  }
  struct pollfd pfd{
    .fd = fd,
    .events = POLLIN
  };
  int rc = poll(&pfd, 1, timeout);
  if(rc > 0){
    rc = read(fd, buffer, count);
    if(rc == -1){
      perror("read");
      return 0;
    }
    return rc;
  }
  return 0;
}

bool echo_test(int fd, N_t message){
    const Command c {Command::EchoN};
    write(fd, (char*)&c, sizeof(Command)); 
    write(fd, (char*)&message, sizeof(N_t));

    Ans answer;
    if(wait_for_read(fd, (uint8_t*)&answer, sizeof(answer), 1000)<=0){
      std::cerr << "[ERROR] failed to read echo ans" << std::endl;
      return false;
    }

    if(answer != Ans::OK){
      std::cerr << "[ERROR] echo result NOK" << std::endl;
      return false;
    }

    N_t message_received;
    if(wait_for_read(fd, (uint8_t*)&message_received, sizeof(N_t), 1000)<=0){
      std::cerr << "[ERROR] failed to read echo message" << std::endl;
      return false;
    }

    if(message_received != message){
      fprintf(stderr, "[ERROR] echo test failed, received: %#x\n", message_received);
      return false;
    }
    return true;
}

bool jumpToAddr(int fd, N_t addr){
      const Command c {Command::JumpToAddr};
      write(fd, (char*)&c, sizeof(Command)); 
      write(fd, (char*)&addr, sizeof(N_t));

      Ans answer;
      if(wait_for_read(fd, (uint8_t*)&answer, sizeof(answer), 1000)<=0){
        std::cerr << "[ERROR] failed to read jump ans" << std::endl;
        return false;
      }

      if(answer != Ans::OK){
      std::cerr << "[ERROR] echo result NOK" << std::endl;
        return false;
      }
      return true;
}

bool WriteNBytes(int fd, const uint8_t bytes[],N_t n){
  assert(n < std::numeric_limits<N_t>::max());
  do{
      // write header
      const Command c {Command::WriteNBytes};
      write(fd, (char*)&c, sizeof(Command)); 
      // write N

      write(fd, (char*)&n, sizeof(N_t));

      // write data
      write(fd, bytes, n);


      // read answer
      Ans answer{Ans::NOK};
      if(wait_for_read(fd, (uint8_t*)&answer, sizeof(Ans), 100000)<=0){
        std::cerr << "[ERROR] failed to read ans" << std::endl;
        return false;
      }

      // if No OK, then try again
      if(answer == Ans::NOK)
          continue;
      break;
  }while(1);
  return true;
}

bool WriteNZeros(int fd, N_t n){
  assert(n < std::numeric_limits<N_t>::max());
  do{
      // write header
      const Command c {Command::WriteNZeros};
      write(fd, (char*)&c, sizeof(Command)); 
      // write N
      write(fd, (char*)&n, sizeof(N_t));

      // read answer
      Ans answer{Ans::NOK};
      if(wait_for_read(fd, (uint8_t*)&answer, sizeof(Ans), 100000)<=0){
        std::cerr << "[ERROR] failed to read ans" << std::endl;
        return false;
      }

      // if No OK, then try again
      if(answer == Ans::NOK)
          continue;
  }while(false);
  return true;
}

bool ReadNAddr(int fd, uint8_t buf[], N_t n, N_t addr){
  assert(n < std::numeric_limits<N_t>::max() && addr < std::numeric_limits<N_t>::max());
  do{
    // write header
    const Command c{Command::ReadNAddr};
    write(fd, (char*)&c, sizeof(Command));
    // write n
    write(fd, (char*)&n, sizeof(N_t));
    // write addr
    write(fd, (char*)&addr, sizeof(N_t));

    // read answer
    Ans answer{Ans::NOK};
    if(wait_for_read(fd, (uint8_t*)&answer, sizeof(Ans), 100000)<=0){
      std::cerr << "[ERROR] failed to read ans" << std::endl;
      return false;
    }
    if(answer == Ans::NOK)
      continue;

    // read results
    if(wait_for_read(fd, buf, n, 100000)<=0){
      std::cerr << "[ERROR] failed to read data" << std::endl;
      return false;
    }
    
  }while(false);
  return true;
}

// helper type for the visitor
template<class... Ts>
struct match : Ts... { using Ts::operator()...; };
template<typename...Func> match(Func...) -> match<Func...>;

template <typename... Ts, typename... Fs>
constexpr decltype(auto) operator| (std::variant<Ts...> const& v, match<Fs...> const& match) {
    return std::visit(match, v);
}

using DataBlocks = std::vector<std::variant<std::vector<uint8_t>, size_t>>;

DataBlocks read_stream(std::istream& stream, const size_t max_cons_zeros = 8){
  DataBlocks data{};
  size_t cons_zeros{0};
  std::for_each(
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>(),
    [&data, &cons_zeros, max_cons_zeros](const char c){
        if(c == 0x00){
          // if last block was zeros, then add this
          if(!data.empty())
            if(auto* zeros = std::get_if<size_t>(&data.back())){
              *zeros += 1;
              return;
            }
          // add to consecutive zeros
          cons_zeros += 1;
          if(cons_zeros <= max_cons_zeros)
            return;
          // if enough consecutive zeros
          data.push_back(std::forward<size_t>(cons_zeros));
          cons_zeros = 0;
        }else{
          // if block exists add to it
          if(!data.empty())
            if(auto* block = std::get_if<std::vector<uint8_t>>(&data.back())){
              if(cons_zeros > 0)
                block->insert(block->end(), cons_zeros, 0);
              block->push_back(c);
              cons_zeros = 0;
              return;
            }
          // else create new block
          std::vector<uint8_t> new_block(cons_zeros, 0);
          new_block.push_back(c);
          data.push_back(std::move(new_block));
          cons_zeros = 0;
        }
    }
  );
  // if zeros
  if(cons_zeros > 0){
    if(!data.empty()){
      // the last block should always be data at this point
      auto& block = std::get<std::vector<uint8_t>>(data.back());
      block.insert(block.end(), cons_zeros,0);      
    }else{
      data.push_back(std::vector<uint8_t>(cons_zeros, 0));
    }
  }
  // for(auto&& block : data){
  //   if(auto* data = std::get_if<std::vector<uint8_t>>(&block)){
  //     std::cout << "data of size: " <<  data->size() << ": ";
  //     for(auto a : *data)
  //       printf("%X ", a);
  //     std::cout << std::endl;
  //   }else if(auto* zeros = std::get_if<size_t>(&block))
  //     std::cout << "zeros of size: " << *zeros << std::endl;
    
  // }
  return data;
}

static bool memvcmp(void *memory, unsigned char val, unsigned int size)
{
    unsigned char *mm = (unsigned char*)memory;
    return (*mm == val) && memcmp(mm, mm + 1, size - 1) == 0;
}

namespace po = boost::program_options;

int uploader_subcommnad(int serial, const std::vector<std::string>& args){
  po::options_description desc("");
  desc.add_options()
    ("help,h", "produces help message")
    ("input-file", po::value<std::string>()->notifier(check_path_exists), "input file");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  try{
    po::store(po::command_line_parser(args).options(desc).positional(p).run(), vm);
    po::notify(vm);
  }catch(po::error& e){
    std::cerr << "[ERROR] " << e.what() << "\n";
    std::cout << desc << "\n";
    return EXIT_FAILURE;
  }

  DataBlocks data_blocks;
  if(vm.count("input-file")){
    std::string filename = vm["input-file"].as<std::string>();

    std::ifstream file(filename, std::ios::binary);

    // stop eating new-lines in binary mode
    file.unsetf(std::ios::skipws);

    // get it's size
    file.seekg(0, std::ios::end);
    std::streampos file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // reserve and read file
    data_blocks = read_stream(file);
  }else{
    if(isatty(STDIN_FILENO)){ // if stdin is not a pipe
      fprintf(stderr, "[ERORR] no input-file provided (or data piped)!");
      return EXIT_FAILURE;
    }
    // read data from stream until end
    data_blocks = read_stream(std::cin);    
  }

    // JumpToAddr 0
    if(!jumpToAddr(serial, 0)){
      return EXIT_FAILURE;
    }

    const size_t CHUNK_SIZE = 4; // 32 bits (half of arduino's serial buffer)

    size_t addr{0};
    
    for(auto&& block : data_blocks){
      bool ret = block | match{
       [serial,&addr](size_t zeros){ // for block of n zeros
        if(!WriteNZeros(serial, zeros)){
          return false;
        }

        // read back the zeros to check if written correctly
        // for(int z=0;z<zeros;z+=EEPROM_PAGE_SIZE){
        //   static std::array<uint8_t, EEPROM_PAGE_SIZE> read_buffer;
        //   ReadNAddr(serial, read_buffer.data(), zeros, addr);
          // if(!memvcmp(read_buffer.data(), 0, read_buffer.size())){
          //   std::cerr << "[ERROR] written " << zeros << " 0s, but read: " << std::endl;
          //   for(auto byte : read_buffer)
          //     std::cerr << (int)byte << ' ';
          //   std::cerr << std::endl;
          //   return false;
          // }
        // }
        
        addr += zeros;
        std::cout << "[INFO] Sent " << zeros << " zeros, that's " << addr << " bytes total" << std::endl;
        return true;
       },
       [=, &addr](const std::vector<uint8_t>& data){ // for block of data
         // iterate data block in PAGE_SIZE offsets
        for(int index=0; index < data.size(); index+=EEPROM_PAGE_SIZE){
          N_t write_n = std::min(EEPROM_PAGE_SIZE, data.size()-index);

          // write page
          if(!WriteNBytes(serial, &data[index], write_n)){
            return false;
          }

          // read back the data block, to check if written correctly
          static uint8_t read_buffer[EEPROM_PAGE_SIZE];
          ReadNAddr(serial, read_buffer, write_n, addr);
          if(memcmp(&data[index], read_buffer, write_n) != 0){
            fprintf(stderr, "[ERROR] read data didn't match written\n");
            for(int i=0;i<write_n;++i)
              fprintf(stderr, "%X:%X\n", (int)read_buffer[i], (int)data[index+i]);
            return false;
          }

          addr += write_n;

          std::cout << "[INFO] Sent " << write_n << " bytes, that's " << addr << " bytes total" << std::endl;
        }
        return true;
       },
      };
      if(!ret){
        return EXIT_FAILURE;
      }
    }

    std::cout << "[INFO] Upload Successfull 🎊" << std::endl;
    return EXIT_SUCCESS;
}

int pretty_print_all(int serial, FILE* out){
  constexpr size_t READ_BYTES = 16;
  constexpr size_t ADDR_HEX_WIDTH = 4; //log16(EEPROM_SIZE);

  std::array<uint8_t, READ_BYTES> last_bytes{};
  bool last_dup{false};

  std::fprintf(out, "┌────┬─────────────────────────┬─────────────────────────┬────────┬────────┐\n");
  
  for(int addr=0;addr<EEPROM_SIZE;addr+=READ_BYTES){
    static std::array<uint8_t, READ_BYTES> bytes;

    if(!ReadNAddr(serial, bytes.data(), READ_BYTES, addr))
      return EXIT_FAILURE;

    if(last_bytes == bytes){
      last_dup = true;
      continue;
    }

    if(last_dup){
      std::fprintf(out, "│%*c│                         ┊                         │        ┊        │\n", (int)ADDR_HEX_WIDTH, '*');
      last_dup = false;
    }

    std::fprintf(out, "│%0*x│ ", (int)ADDR_HEX_WIDTH, addr);

    for(int i=0;i<READ_BYTES/2;++i)
      std::fprintf(out, "%02x ", bytes[i]);
    std::fprintf(out, "┊ ");
    for(int i=READ_BYTES/2;i<READ_BYTES;++i)
      std::fprintf(out, "%02x ", bytes[i]);
    std::fprintf(out, "│");
    for(int i=0;i<READ_BYTES/2;++i)
      if (bytes[i]==0)
        std::fprintf(out, "⋄");
      else if(32<bytes[i] && bytes[i]<=127)
        std::fprintf(out, "%c", (char)bytes[i]);
      else
        std::fprintf(out, "×");
    std::fprintf(out, "┊");
    for(int i=READ_BYTES/2;i<READ_BYTES;++i)
      if (bytes[i]==0)
        std::fprintf(out, "⋄");
      else if(32<bytes[i] && bytes[i]<=127)
        std::fprintf(out, "%c", (char)bytes[i]);
      else
        std::fprintf(out, "×");
    std::fprintf(out, "│\n");

    last_bytes = bytes;
  }
  std::fprintf(out, "└────┴─────────────────────────┴─────────────────────────┴────────┴────────┘\n");
  return EXIT_SUCCESS;
}

int raw_print_all(int serial, FILE* out){
  for(int addr=0;addr<EEPROM_SIZE; addr+=EEPROM_PAGE_SIZE){
    static uint8_t bytes[EEPROM_PAGE_SIZE];

    if(!ReadNAddr(serial, bytes, EEPROM_PAGE_SIZE, addr))
      return EXIT_FAILURE;
    
    std::fwrite(bytes, sizeof(uint8_t), sizeof(bytes), out);
  }
  return EXIT_SUCCESS;
}

int reader_subcommand(int serial, const std::vector<std::string>& args){
  po::options_description desc("");
  desc.add_options()
    ("help,h", "produces help message")
    ("raw,r", "prints raw bytes, by default prints pretty formatted output")
    ("output,o", po::value<std::string>(), "output file path");

  po::positional_options_description p;
  p.add("output", 1);

  po::variables_map vm;
  try{
    po::store(po::command_line_parser(args).options(desc).positional(p).run(), vm);
    po::notify(vm);
  }catch(po::error& e){
    std::cerr << "[ERROR] " << e.what() << "\n";
    std::cout << desc << "\n";
    return EXIT_FAILURE;
  }

  FILE* output = stdout;
  if(vm.count("output")){
    output = std::fopen(vm["output"].as<std::string>().c_str(), "r");
  }

  int ret = vm.count("raw") ? raw_print_all(serial, output) : pretty_print_all(serial, output);  

  if(vm.count("output"))
    std::fclose(output);

  return ret;
}

int main(int argc, char** argv){
  po::options_description global_desc("Global options");
  global_desc.add_options()
    ("debug,d", "Turn on debug output")
    ("test,t", "send echo before any transmission to test connection")
    ("port,p", po::value<std::string>()->required()->notifier(check_path_exists), "EEprogm programator port")
    ("baudrate,b", po::value<int>()->default_value(DEFAULT_BAUDRATE), "baudrate for the port")
    ("command", po::value<std::string>()->default_value("reader")->required(), "command to execute ('reader' or 'upload')")
    ("subargs", po::value<std::vector<std::string>>(), "Arguments for command");

  po::positional_options_description pos{};
  pos.add("command",1)
     .add("subargs",-1);

  po::variables_map vm;
  po::parsed_options parsed = po::command_line_parser(argc, argv)
    .options(global_desc)
    .positional(pos)
    .allow_unregistered()
    .run();
  try{
    po::store(parsed, vm);
    po::notify(vm);
  }catch(po::error& e){
    std::cerr << "[ERROR] " << e.what() << "\n";
    std::cout << global_desc << "\n";
    return EXIT_FAILURE;
  }

  int serial = setup_serial(vm["port"].as<std::string>(), vm["baudrate"].as<int>());
  if(serial == -1){
      return EXIT_FAILURE;
  }

  // ECHO
  if(vm.count("test")){
    if(echo_test(serial, 0xDEAD) && echo_test(serial, 0xBEAF))
      std::cout << "[INFO] ECHO success" << std::endl;
    else
      return EXIT_FAILURE;
  }
  
  auto cmd = vm["command"].as<std::string>();
  std::vector<std::string> subargs = po::collect_unrecognized(parsed.options, po::include_positional);
  subargs.erase(subargs.begin());

  // auto subargs = vm["subargs"].as<std::vector<std::string>>(); 
  if(cmd == "upload")
    return uploader_subcommnad(serial, subargs);
  else if(cmd == "reader")
    return reader_subcommand(serial, subargs);


}
