// #include "BluetoothSerial.h"

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled!
// #endif

#include "../shared/commands.h"
#include "../shared/config.h"

const int 
  SHIFT_DATA = 32,
  SHIFT_CLK = 33,
  SHIFT_LATCH = 25,
  WRITE_EN = 26;
const int DATA_PINS[] = {13, 4, 21, 5, 18, 23, 19, 22};


void setAddress(int address, bool outputEnable){
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x0 : 0x80));
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);  
}

byte readEEPROM(int address){
  setAddress(address, /*outputEnable*/ true);
  byte data{};
  for(int pin=7; pin >= 0; --pin){
    pinMode(DATA_PINS[pin], INPUT);
    data = (data << 1) + digitalRead(DATA_PINS[pin]);
  }
  return data;
}

void writeByteEEPROM(int address, byte data){
  setAddress(address, false);

  for(int pin = 0; pin <= 7; ++pin){
    pinMode(DATA_PINS[pin], OUTPUT);
    digitalWrite(DATA_PINS[pin], data & 1);
    data >>= 1;
  }

  digitalWrite(WRITE_EN, LOW);
  delayMicroseconds(1); // min: 100ns
  digitalWrite(WRITE_EN, HIGH);
  delay(10); // min: 10ms
}

inline 
void writePageEEPROM(int address, const byte pageBytes[], const byte byte_n=EEPROM_PAGE_SIZE){
  for(int offset=0;offset<byte_n;++offset){
    byte data = pageBytes[offset];
    setAddress(address+offset, false);
    for(int pin = 0; pin <= 7; ++pin){
      pinMode(DATA_PINS[pin], OUTPUT);
      digitalWrite(DATA_PINS[pin], (data>>pin)&1);
    }

    digitalWrite(WRITE_EN, LOW);
    delayMicroseconds(10); // min: 100ns, max: 150us
    digitalWrite(WRITE_EN, HIGH); // min: 50ns
  }
  delay(10); // min: 10ms
}
inline 
void fillPageEEPROM(int address, const byte fill = 0, const byte byte_n=EEPROM_PAGE_SIZE){
  for(int pin = 0; pin <= 7; ++pin){
    pinMode(DATA_PINS[pin], OUTPUT);
    digitalWrite(DATA_PINS[pin], (fill>>pin)&1);
  }

  for(int offset=0;offset<byte_n;++offset){
    setAddress(address+offset, false);
    delayMicroseconds(10);

    digitalWrite(WRITE_EN, LOW);
    delayMicroseconds(10); // min: 100ns, max: 150us
    digitalWrite(WRITE_EN, HIGH); // min: 50ns
  }
  delay(10); // min: 10ms
}


// Callback function to handle authentication
// void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  // if (event == ESP_SPP_MODE_CB && param->sec_id.sec_mask == ESP_SPP_SEC_AUTHENTICATE) {
  //   Serial.println("Device successfully authenticated.");
  // }
// }

void setup(){  
  digitalWrite(WRITE_EN, HIGH); // set WRITE ENABLE high initially
  pinMode(WRITE_EN, OUTPUT);

  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);

  Serial.begin(DEFAULT_BAUDRATE);
  Serial.setTimeout(1000);
}

template<typename T>  inline 
size_t SerialWriteType(T value){
  return Serial.write((byte*)&value, sizeof(T));
}
template<typename T>  inline 
size_t SerialReadType(T *value){
  return Serial.readBytes((byte*)value, sizeof(T));
}

inline
bool parseCommand(){
  int available = Serial.available();
  if(available <= 0)
    return true;

  static size_t addr{0};
  static byte data[EEPROM_SIZE];
  
  Command c;
  SerialReadType(&c);

  switch (c){
    case Command::WriteNBytes:{
        N_t n;
        if(SerialReadType(&n) != sizeof(n))
          return false;

        int read = 0;
        while(read < n){
          if(!Serial.available())
            return false;
          read += Serial.readBytes(&data[addr + read], n - read);
        }

        // first fill the previous page
        const size_t page_offset = addr & EEPROM_PAGE_ADDR_MASK;
        if(page_offset != 0){
          const size_t written = (n > (EEPROM_PAGE_SIZE - page_offset))?(EEPROM_PAGE_SIZE - page_offset) : n;
          n-=written;
          writePageEEPROM(addr, &data[addr], written);
          addr += written;
        }
        
        while(n > 0){
          size_t written = min((N_t)EEPROM_PAGE_SIZE, n);
          n-=written;
          writePageEEPROM(addr, &data[addr], written);
          addr += written;
        }

        SerialWriteType(Ans::OK);
    }break;
    case Command::WriteNZeros:{
        N_t n;
        if(SerialReadType(&n) != sizeof(n))
          return false;

        memset(&data[addr], 0, n);
        
        // first fill the previous page
        const size_t page_offset = addr & EEPROM_PAGE_ADDR_MASK;
        if(page_offset != 0){
          const size_t written = (n > (EEPROM_PAGE_SIZE - page_offset))?(EEPROM_PAGE_SIZE - page_offset) : n;
          n-=written;
          fillPageEEPROM(addr, 0, written);
          addr += written;
        }

        while(n > 0){
          size_t written = min((N_t)EEPROM_PAGE_SIZE, n);
          n-=written;
          fillPageEEPROM(addr, 0, written); // fill page with 0
          addr += written;
        }
        
        SerialWriteType(Ans::OK);
    }break;
    case Command::ReadNAddr:{
        N_t n;
        if(SerialReadType(&n) != sizeof(n))
          return false;
        N_t read_addr;
        if(SerialReadType(&read_addr) != sizeof(read_addr))
          return false;

        SerialWriteType(Ans::OK);

        for(int i=0;i<n;++i)
          SerialWriteType(readEEPROM(read_addr+i));
      
    }break;
    case Command::EchoN:{
      N_t n;
      if(SerialReadType(&n) != sizeof(n))
        return false;
      
      // OK
      SerialWriteType(Ans::OK);
      // Echo 
      SerialWriteType(n);
    }break;
    case Command::JumpToAddr:{
      N_t new_addr;
      if(SerialReadType(&new_addr) != sizeof(new_addr))
        return false;
      addr = new_addr;
      // OK
      SerialWriteType(Ans::OK);
    }break;
    default: // unimplemented commands
      return false;
  }
  return true;
}


void loop(){
  if (!parseCommand())
    SerialWriteType(Ans::NOK);
  Serial.flush();
}

