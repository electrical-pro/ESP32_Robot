#ifndef uniPar_H
#define uniPar_H

#include<Arduino.h>
#include<PID.h>
#include<WebSocketsServer.h>

#define EEPROM_ADR_PID_WRITTEN 200
#define EEPROM_ADR_PID 201

#define EEPROM_WRITTEN 123 // Flag to check if EEPROM has been initialised

// #define PAR_READ 0
// #define PAR_WRITE 1
//
// void readPIDParameters(PID* pid);
// void writePIDParameters(PID* pid);

typedef void (*funPointer) ();

enum tag {t_u8,t_u16,t_u32,t_i8,t_i16,t_i32,t_f,t_d,t_fun};
const uint8_t tagSize[] = {1,2,4,1,2,4,4,8,0};

typedef union {
  uint8_t arr[6];
  struct {
    uint8_t grp;
    uint8_t cmd;
    union {
      float val;
      uint8_t valU8[4];
    };
  }  __attribute__((packed));
} cmd;

// typedef union {
//   uint8_t u8[4];
//   float f;
// } floatToByte;


class par {
public:
  // uint8_t* p;
  union {
    uint8_t * p_u8;
    uint16_t * p_u16;
    uint32_t * p_u32;
    int8_t * p_i8;
    int16_t * p_i16;
    int32_t * p_i32;
    float * p_f;
    double * p_d;
    funPointer p_fun;
  };

  int address;
  uint8_t cmd;
  uint8_t tag;

  static int addressCounter;
  static uint8_t cmdCounter;


  // par(uint8_t* _p, uint8_t _tag, uint8_t _cmd, int _address);
  // par(uint8_t* _p, uint8_t _tag);
  par(uint8_t* _p); // Overload constructor for different variable types
  par(float* _p);
  par(funPointer _p);

  void read(void);
  void write(void);
  float getFloat(void);
  void setFloat(float f);

private:
  void assignAddress(void);
};

class parList {
public:
  parList(par* _l);
  void sendList(WebSocketsServer *wsServer);
  void parseMessage(cmd c);
  void read(void);
  void write(void);
  void set(uint8_t cmd, float f);

  static uint8_t groupCounter;
  uint8_t groupNo;
  uint8_t numPar;

  uint16_t flagAddress;
private:
  par* l;

};
#endif
