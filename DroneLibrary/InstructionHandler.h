#ifndef INSTRUCTION_HANDLER_H_
#define INSTRUCTION_HANDLER_H_

#include <Arduino.h>

class Message {
public:
    bool initiated = false;
    uint8_t type, length;

    // Initiate message without a type
    Message();

    // Initiate message with a type
    Message(uint8_t type);

    // sets type and length, and initiated = true
    bool set(uint8_t type);

    // Resets messege
    // initiated = false.
    void reset();
};

class InstructionHandler {
    Message message;
    bool reading = false;
    bool acquiredData = false;
    uint8_t readIndex = 0;
    uint8_t readBuffer[31];
    uint8_t startMarker = 33;

public:
    bool read();
    uint8_t getData(void* out);
    void write(uint8_t* data, uint8_t type);
    void acknowledge(uint8_t messageType);
};
#endif