// #include "BluetoothSerial.h"

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled!
// #endif

#include "../shared/commands.h"

const int 
  SHIFT_DATA = 32,
  SHIFT_CLK = 33,
  SHIFT_LATCH = 25,
  WRITE_EN = 26;
const int DATA_PINS[] = {13, 4, 21, 5, 18, 23, 19, 22};

const size_t EEPROM_SIZE = 32768;
const size_t EEPROM_PAGE_SIZE = 64;
const size_t EEPROM_PAGE_ADDR_MASK = EEPROM_PAGE_SIZE-1;

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
void writePageEEPROM(int address, byte pageBytes[], const byte byte_n=EEPROM_PAGE_SIZE){
  for(int offset=0;offset<byte_n;++offset){
    byte data = pageBytes[offset];
    setAddress(address+offset, false);
    for(int pin = 0; pin <= 7; ++pin){
      pinMode(DATA_PINS[pin], OUTPUT);
      digitalWrite(DATA_PINS[pin], data & 1);
      data >>= 1;
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

void printContents(){
  byte last[16];
  bool last_dup{false};
  bool duplicate{false};
  for(int base = 0;base <= 0x7ff0; base += 16){
    byte data[16];
    last_dup = duplicate;
    duplicate = true;
    for(int offset = 0; offset <= 15; ++offset){
      data[offset] = readEEPROM(base + offset);
      if(data[offset]!=last[offset])
        duplicate = false;
    }
    if(duplicate){
      continue;
    }
    if(last_dup)
      Serial.println("*");
      
    char buf[80];
    sprintf(buf, "%03x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
       base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
             data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
    memcpy(last, data, sizeof(data));
  }
}


/*
const byte code[] = {
  0xa9, 0xff,       // lda $ff (LOAD 0xFF to A)
  0x8d, 0x02, 0x60, // sta $6002 (STORE A to addr $6002)

  0xa9, 0x55,       // lda $55
  0x8d, 0x00, 0x60, // sta $6000

  0xa9, 0xaa,       // lda $aa
  0x8d, 0x00, 0x60, // sta $6000

  0x4c, 0x05, 0x80  // jmp $8005
};
*/

void setup(){  
  digitalWrite(WRITE_EN, HIGH); // set WRITE ENABLE high initially
  pinMode(WRITE_EN, OUTPUT);

  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);


  Serial.begin(115200);
  Serial.setTimeout(1000);

  // esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  // esp_bt_pin_code_t pin_code = {'4', '6', '2', '6',}; 
  // esp_bt_gap_set_pin(pin_type, 4, pin_code);
  
  // BluetoothSerial SerialBT;
  // SerialBT.register_callback(btCallback);
  // SerialBT.begin("EEPROM_programmator");

  // printContents();
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
          int available  = Serial.available();
          if(!available)
            return false;
          read += Serial.readBytes(data + addr + read, n - read);
        }


        // first fill the previous page
        size_t page_addr = addr & EEPROM_PAGE_ADDR_MASK;
        if(page_addr != 0){
          size_t write = (n > (EEPROM_PAGE_SIZE - page_addr))?(n-(EEPROM_PAGE_SIZE - page_addr)) : n;
          n-=write;
          writePageEEPROM(addr, &data[addr], write);
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
        size_t page_addr = addr & EEPROM_PAGE_ADDR_MASK;
        if(page_addr != 0){
          size_t write = (n > (EEPROM_PAGE_SIZE - page_addr))?(n-(EEPROM_PAGE_SIZE - page_addr)) : n;
          n-=write;
          writePageEEPROM(addr, &data[addr], write);
        }
        while(n > 0){
          size_t page_addr = addr & EEPROM_PAGE_ADDR_MASK;
          size_t written = min((N_t)EEPROM_PAGE_SIZE, n)-page_addr;
          n-=written;
          fillPageEEPROM(addr, 0, written); // fill page with 0
          addr += written;
        }
        
        
        SerialWriteType(Ans::OK);
    }break;
    case Command::CheckBytes:{
      
      for(int i = 0;i <= addr; ++i){
        if(readEEPROM(i) != data[i])
          return false;
      }
      // Success
      SerialWriteType(Ans::OK);
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
  }
  return true;
}


void loop(){
  if (!parseCommand())
    SerialWriteType(Ans::NOK);
  Serial.flush();
}
