/* Commands:
    0 : write N bytes : OK 
    1 : write N zeros : OK 
    2 : skip N bytes  : OK 
    3 : jump to N addr
    *Where N is a 32bit unsigned int
*/

#include <cstddef>
#include <cstdint>

using Ans_t = char;
enum class Ans : Ans_t{
    NOK = '-',
    OK = '+'
};

enum class Command : char{
    // 
    WriteNBytes = 0x00,
    WriteNZeros,
    SkipNBytes,
    JumpToNAddr,// (N_t)n
    CheckBytes, // 
    EchoN,      // (N_t)n
    FillNBytes, // (N_t)n, (char)fill
    Reset,      // Resets internal buffer and position
    //
    // GetSize // perform device size check
};

using N_t = uint16_t;

inline void execute(Command c, int *rx(char*, size_t), int *tx(const char*, size_t)){
    switch (c){
    case Command::WriteNBytes:
        N_t n;
        rx((char*)&n,1);
        
        
    case Command::WriteNZeros:
    case Command::SkipNBytes:
    case Command::JumpToNAddr:
      break;
    }
}
