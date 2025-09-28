#include <cstddef>

const size_t DEFAULT_BAUDRATE = 115200;//460800; // 460800 was too fast, for the messages to fit in arduino's buffer

const size_t EEPROM_SIZE = 0x8000;
const size_t EEPROM_PAGE_SIZE = 64;
const size_t EEPROM_PAGE_ADDR_MASK = EEPROM_PAGE_SIZE-1;

