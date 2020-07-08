#include <par.h>
#include <EEPROM.h>
#include <Streaming.h>

/*
How to pass objects to be modified?
  For PID, this is simple: pass array of PID objects, or a single PID object.

How to define a list of parameters to be stored?
  For example, for PID this would be: type (uint8_t), P/I/D/N/min/max(/R) (float)
  Every parameters should have a command (integer / character).

Proposal: use parameter objects.
Properties: pointer, type/tag, command, EEPROM address (maybe use static counter)
Functions: read/write EEPROM, receive command, send command
Sending a command is easy: get the value that the pointer corresponds to,
compose a message with a header, maybe add checksum, and return an array and length.
Receiving is a bit harder: how to (efficiently) find which parameter/pointer corresponds to the command?
Easiest method would be to have an array of parameters, which can be indexed.
Makes sending a bit harder: how to send a single command. However, when something is sent,
all parameters are sent.

In this way, can a single function be used for reading/writing from/to EEPROM?
Why not: as long as all variable types are covered, it should work.
However, a (group of) parameter(s) might need an update function.

Maybe add both command parameters to parameter definition?


To do:
Check if address counter is smaller than EEPROM size
Same for command
Add option for manually specifying command

Idea:
Add a save command, to save all parameters (both start and end address are known);
*/



int par::addressCounter = 300; // Starting address
uint8_t par::cmdCounter = 0;
uint8_t parList::groupCounter = 0;

parList::parList(par* _l) {
  l = _l;
  groupNo = groupCounter++;
  numPar = par::cmdCounter;
  par::cmdCounter = 0;

  flagAddress = par::addressCounter++;
}

void parList::sendList(WebSocketsServer *wsServer) {
  uint8_t buf[6];
  cmd c;

  c.grp = groupNo;

  for (uint8_t i=0; i<numPar; i++) {
    Serial << groupNo << "\t" << l[i].cmd << "\t" << l[i].getFloat() << endl;
    c.cmd = l[i].cmd;
    c.val = l[i].getFloat();
    wsServer->sendBIN(0,c.arr,6);
  }
}

void parList::write(void) {
  for (uint8_t i=0; i<numPar; i++) {
    l[i].write();
  }
  EEPROM.write(flagAddress, EEPROM_WRITTEN);
  EEPROM.commit();
}

void parList::read(void) {
  uint8_t flag = EEPROM.read(flagAddress);
  if (flag==EEPROM_WRITTEN) { // Only read if EEPROM has been written to previously
    for (uint8_t i=0; i<numPar; i++) {
      l[i].read();
    }
  }
}

void parList::set(uint8_t cmd, float f) {
  if (l[cmd].tag == t_fun) {
    l[cmd].p_fun();
  } else {
    l[cmd].setFloat(f);
  }
}

// Parameter constructors for different variable types
par::par(uint8_t* _p) {
  p_u8 = _p;
  tag = t_u8;
  assignAddress();
}

par::par(float* _p) {
  p_f = _p;
  tag = t_f;
  assignAddress();
}

par::par(funPointer _p) {
  p_fun = _p;
  tag = t_fun;
  assignAddress();
}

void par::assignAddress(void) {
  cmd = cmdCounter++;
  address = addressCounter;
  addressCounter += tagSize[tag];
}

void par::read(void) {
  switch(tag) {
    case t_u8:
      *p_u8 = EEPROM.read(address);
      break;
    case t_f:
      *p_f = EEPROM.readFloat(address);
      break;
  }
  // Serial.println(EEPROM.read(address));
}

void par::write(void) {
  switch(tag) {
    case t_u8:
      EEPROM.write(address, *p_u8);
      break;
    case t_u16:
      EEPROM.writeUShort(address, *p_u16);
      break;
    case t_u32:
      break;
    case t_i8:
      break;
    case t_i16:
      break;
    case t_i32:
      break;
    case t_f:
      EEPROM.writeFloat(address, *p_f);
      break;
    case t_d:
      break;

  }
}

float par::getFloat(void) {
  switch (tag) {
    case t_u8:
      return ((float) *p_u8);
      break;
    case t_f:
      return ((float) *p_f);
      break;
  }
}

// Update value based on pointer
void par::setFloat(float f) {
  switch (tag) {
    case t_u8:
      *p_u8 = (uint8_t) f;
      break;
    case t_f:
      *p_f = f;
      break;
  }
}
// Use binary values or text for messages?
// Send everything as float?

// Add 2nd class for making groups of commands.
// Properties: groupNo, number of items, array of parameters
// EEPROM bit/byte to indicate if settings have been stored
// (pointer to) function for updating
// Functions: read / write group from/to EEPROM,
// Specify group as array of adresses, then loop through them.


// How to implement a save button? I.e. how to define an action that has no variable, but executes a function?
// Maybe set cmd 0 to save all variables, and cmd 1 to return all variables. Or, 254 and 255.

// When receiving a message, all "groups"/lists should receive it.
// Or, do the "group" handling in main. Also allows for implementing custom requests.

/* So, the proposed workflow is:
- Create multiple arrays of grouped parameter objects
- Pass them to corresponding list objects. Or, use non-class function.
- Have a command for sending all variable values
- For incoming message, first byte is command 1. Switch statement points to correct group of variables.
-
*/

// Maybe have a list of parameter groups? Allows to nicely index when command is received.

// uint8_t par::makeMessageBin(uint8_t* p) {
//   // Command (2 bytes), value (1-4 bytes)
//   // How to handle different variable types? Send everything as float, or send variable type as well
//   uint8_t length = 0;
//
//   p[0] = command;
// }
//
// void par::makeMessageText(char* c) {
//
// }
