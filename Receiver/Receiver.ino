#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <InstructionHandler.h>
#include <CountdownTimer.h>

// #define DEBUG

// #ifdef DEBUG
//   #define DEBUG_PRINT(x) do { Serial.print(__FILE__); Serial.print(":"); Serial.print(__LINE__); Serial.print(" -> "); Serial.println(x); } while(0)
// #endif

#define CE_PIN 7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN, 4000000);

InstructionHandler instructionHandler;
uint8_t readBuffer[31];

RadioMessage messageOut, messageIn;

long previousTime;

bool sending = true;

uint8_t connectionStatus = 1;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    if (!radio.begin()){
        Serial.println(F("Radio hardware not responding!"));
        while (true);
    }
    if(!configureRadio(radio)) {
        Serial.println("Radio configuration failed");
        while (true);
    }
    radio.openWritingPipe(RECEIVER_ADDRESS);
    radio.openReadingPipe(1, DRONE_ADDRESS);
    radio.startListening();

    previousTime = micros();
    
    #ifdef DEBUG
      DEBUG_PRINT("Setup complete!");
      delay(5000);
    #endif
}

void loop()
{
    #ifdef DEBUG
      DEBUG_PRINT("loop start");
      Serial.print("Sending: ");
      Serial.println(((sending) ? "true" : "false"));
    #endif

    float deltaTime = (float)(micros() - previousTime) / 1000000;
    previousTime = micros();
    if (deltaTime == 0) {
        deltaTime = 0.000001;
    }

    connectionStatus = 1;

    // Serial input
    if (instructionHandler.read())
    {
        uint8_t messageType = instructionHandler.getData(readBuffer);
        messageOut.messageType = messageType;
        memcpy(messageOut.dataBuffer, readBuffer, sizeof(messageOut.dataBuffer));
        
        #ifdef DEBUG
          printRadioMessage(messageOut);
        #endif

        bool result = false;
        if (sending) 
          result = send();
        if (result) {
          sending = false;
          instructionHandler.acknowledge(messageType);
        }


        switch (messageType) {
        case _MSG_CONTROLLER_INPUT:
            if (result) {
                #ifdef DEBUG
                  ControllerInstructions controller;
                  memcpy(&controller, messageOut.dataBuffer, sizeof(controller));
                  DEBUG_PRINT("x:");
                  Serial.print((float)controller.x / 127);
                  Serial.print(" y:");
                  Serial.print((float)controller.y / 127);
                  Serial.print(" power:");
                  Serial.println((float)controller.power / 127);
                #endif
                break;
            }
        #ifdef DEBUG
            else {
              receiverPrint("Faild to send: Controller input");
              }
            break;
        case _MSG_SET_PID_V:
            if (result)
              receiverPrint("Sent: PID-Velocity");
            else
              receiverPrint("Failed to send: PID-Velocity");
          break;
        case _MSG_SET_PID_P:
          if (result)
              receiverPrint("Sent: PID-Pitch");
            else
              receiverPrint("Failed to send: PID-Pitch");
          break;
        case _MSG_SET_PID_R:
            if (result)
              receiverPrint("Sent: PID-Roll");
            else
              receiverPrint("Failed to send: PID-Roll");
          break;
        case _MSG_SET_TARGET_RANGES:
          if (result)
              receiverPrint("Sent: Target ranges");
            else
              receiverPrint("Failed to send: Target ranges");
          break;
        case _MSG_ACTIVATE:
          if (result)
            receiverPrint("Sent: Activation request");
          else 
            receiverPrint("Failed to send: Activation request");
          break;
        case _MSG_DEACTIVATE:
          if (result)
            receiverPrint("Sent: Deactivation request");
          else 
            receiverPrint("Failed to send: Deactivation request");
          break;
        case _MSG_REQUEST_PID_V:
        case _MSG_REQUEST_PID_P:
        case _MSG_REQUEST_PID_R:
        case _MSG_REQUEST_TARGET_RANGES:
          if (result)
            receiverPrint("Sent: Request");
          else
            receiverPrint("Failed to send: Request");
          break;
        // TODO: Add other messages
        default:
            if (result)
              receiverPrint("Sent: TYPE=UNKNOWN");
            else
              receiverPrint("Failed to send: TYPE=UNKNOWN");
            break;
        #endif
        }
    }

    // Radio input
    if (radio.available()) {
        sending = true;

        connectionStatus = connectionStatus | 0b010;
        
        radio.read(&messageIn, sizeof(messageIn));

        #ifdef DEBUG
          Serial.println("Radio received");
          printRadioMessage(messageIn);
        #endif

        switch (messageIn.messageType) {
          case _MSG_DRONE_LOG:
            dronePrint((const char*)messageIn.dataBuffer);
            break;
          case _MSG_ACTIVATE:
            connectionStatus = connectionStatus | 0b100;
            break;
          case _MSG_DEACTIVATE:
            break;
          default:
            #ifdef DEBUG
              DEBUG_PRINT("begin: write");
            #endif
            instructionHandler.write(messageIn.dataBuffer, messageIn.messageType);
            #ifdef DEBUG
              DEBUG_PRINT("end: write");
            #endif
            break;
          }
      }
      
      #ifdef DEBUG
        DEBUG_PRINT("begin: write");
        #endif
        instructionHandler.write(&connectionStatus, _MSG_CONNECTION_STATUS);
        instructionHandler.write((uint8_t*)&deltaTime, _MSG_RECEIVER_DELTATIME);
      #ifdef DEBUG
        DEBUG_PRINT("end: write");
      #endif
}

bool send()
{
    #ifdef DEBUG
      // Serial.println("trying to send");
    #endif

    radio.stopListening();
    bool result = radio.write(&messageOut, sizeof(messageOut));
    radio.startListening();


    #ifdef DEBUG
      // if (result)
      //   Serial.println("successfully sent");
      // else
      //   Serial.println("failed to send");
    #endif
    
    return result;
}

void receiverPrint(const char* message) {
  #ifdef DEBUG
    DEBUG_PRINT("[Receiver]");
  #else
    Serial.print("[Receiver] ");
  #endif
  Serial.println(message);
}

void dronePrint(const char* message) {
  Serial.print("[Drone] ");
  Serial.println(message);
}

#ifdef DEBUG
void printRadioMessage(RadioMessage message) {
    #ifdef DEBUG
      DEBUG_PRINT("Message type: ");
    #else
      Serial.print("Message type: ");
    #endif
    Serial.println(message.messageType);
    Serial.print("Data: ");
    for (int i = 0; i < sizeof(message.dataBuffer); i++) {
        Serial.print((int)message.dataBuffer[i]);
        Serial.print(" ");
    }
    Serial.print("Length: ");
    Serial.println((int)sizeof(message));
}
#endif