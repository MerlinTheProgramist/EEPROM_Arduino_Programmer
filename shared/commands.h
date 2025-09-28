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
    JumpToAddr,// (N_t)n
    EchoN,      // (N_t)n
    FillNBytes, // (N_t)n, (char)fill
    Reset,      // Resets internal buffer and position
    ReadNAddr,  // (N_t)n, (N_t)addr
    //
    // GetSize // perform device size check
};

using N_t = uint16_t;
