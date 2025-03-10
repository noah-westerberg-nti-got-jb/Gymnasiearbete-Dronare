#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <MotorControl.h>
#include <Vectors.h>
#include <Orientation.h>
#include <RadioSendStack.h>

#define DEBUG

#define CE_PIN 7
#define CSN_PIN 6

// Radio
RF24 radio(CE_PIN, CSN_PIN, 4000000);
uint8_t readBuffer[32];
RadioSendStack sendStack;
RadioMessage messageIn, messageOut;

// Motors
#define MOTOR_TL_Pin 5
#define MOTOR_TR_Pin 4
#define MOTOR_BR_Pin 3
#define MOTOR_BL_Pin 2
uint8_t motorPowerTL = 0, motorPowerTR = 0, motorPowerBR = 0, motorPowerBL = 0;
MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);


// PID-values
float targetVelocity;
float maxVelocity;
PID_Instructions PID_Velocity;

float targetPitch;
float maxPitch;
PID_Instructions PID_Pitch;

float targetRoll;
float maxRoll;
PID_Instructions PID_Roll;


// Delta time
unsigned long previousTime;
float deltaTime;
void setDeltaTime();


// Spatial orientation/acceleration/velocity
const int MPU = 0x68;
Orientation orientation(MPU);


bool activated = false;

void setup() {
    #ifdef DEBUG
      Serial.begin(115200);
      while (!Serial);
    #endif

    // Set up radio
    while(!radio.begin()) {
      #ifdef DEBUG
        Serial.println("radio hardrware issue");
      #endif
    }
    while(!configureRadio(radio));
     {
      #ifdef DEBUG
        Serial.println("radio config issue");
      #endif
     }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);
    radio.startListening();

    // Set up motor controller
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(&PID_Velocity);
    motorController.setPitchConstants(&PID_Pitch);
    motorController.setRollConstants(&PID_Roll);

    pinMode(MOTOR_TL_Pin, OUTPUT);
    pinMode(MOTOR_TR_Pin, OUTPUT);
    pinMode(MOTOR_BR_Pin, OUTPUT);
    pinMode(MOTOR_BL_Pin, OUTPUT);
    
    // Initial time
    previousTime = micros();

    radioLog("Connected", true);

    #ifdef DEBUG
      Serial.println("completed setup");
    #endif
}

void loop() {
    setDeltaTime();
    
    // Radio read
    if (radio.available() && millis() % 20 != 0) {
        #ifdef DEBUG
          Serial.println("radio available");
        #endif

        // scope variables
        TargetRangeInstructions targetRanges;
        uint8_t power;
        float time;
        
        radio.read(&readBuffer, sizeof(readBuffer));
        memcpy(&messageIn, &readBuffer, sizeof(messageIn));

        #ifdef DEBUG
          printRadioMessage(messageIn);
        #endif

        switch (messageIn.messageType) {
            case _MSG_CONTROLLER_INPUT:
                controllerInstructions controller;
                memcpy(&controller, messageIn.dataBuffer, sizeof(controller));
                
                #ifdef DEBUG
                  Serial.print("x:");
                  Serial.print((float)controller.stick_X / 127);
                  Serial.print(" y:");
                  Serial.print((float)controller.stick_Y / 127);
                  Serial.print(" power:");
                  Serial.println((float)controller.power / 127);
                #endif

                targetVelocity = maxVelocity * (float)controller.power / 127;
                targetPitch = maxPitch * ((float)controller.stick_X / 127);
                targetRoll = maxRoll * ((float)controller.stick_Y / 127);
                break;
            case _MSG_ACTIVATE:
                actiiivate();
                break;
            case _MSG_DEACTIVATE:
                deactivate();
                break;
            case _MSG_SET_PID_V:
                memcpy(&PID_Velocity, messageIn.dataBuffer, sizeof(PID_Velocity));

                #ifdef DEBUG
                  Serial.print("PID-V: p:");
                  Serial.print(PID_Velocity.k_p);
                  Serial.print(" i:");
                  Serial.print(PID_Velocity.k_i);
                  Serial.print(" d:");
                  Serial.println(PID_Velocity.k_d);
                #endif

                break;
            case _MSG_SET_PID_P:
                memcpy(&PID_Pitch, messageIn.dataBuffer, sizeof(PID_Pitch));               
                break;
            case _MSG_SET_PID_R:
                memcpy(&PID_Roll, messageIn.dataBuffer, sizeof(PID_Roll));
                break;
            case _MSG_REQUEST_PID_V:
                memcpy(messageOut.dataBuffer, &PID_Velocity, sizeof(PID_Velocity));
                messageOut.messageType = _MSG_SET_PID_V;
                sendStack.push(messageOut);

                #ifdef DEBUG
                  Serial.println("Requesting pid-v");
                #endif

                break;
            case _MSG_REQUEST_PID_P:
                memcpy(messageOut.dataBuffer, &PID_Pitch, sizeof(PID_Pitch));
                messageOut.messageType = _MSG_SET_PID_P;
                sendStack.push(messageOut);
                break;
            case _MSG_REQUEST_PID_R:
                memcpy(messageOut.dataBuffer, &PID_Roll, sizeof(PID_Roll));
                messageOut.messageType = _MSG_SET_PID_R;
                sendStack.push(messageOut);
                break;
            case _MSG_SET_TARGET_RANGES:
                memcpy(&targetRanges, messageIn.dataBuffer, sizeof(targetRanges));
                maxVelocity = targetRanges.verticalVelocityMax;
                maxPitch = targetRanges.pitchMax;
                maxRoll = targetRanges.rollMax;
                break;
            case _MSG_REQUEST_TARGET_RANGES:
                targetRanges = {maxPitch, maxRoll, maxVelocity};
                memcpy(messageOut.dataBuffer, &targetRanges, sizeof(targetRanges));
                messageOut.messageType = _MSG_SET_TARGET_RANGES;
                sendStack.push(messageOut);
                break;
            default:
                radioLog("Error interpreting messageType", false);
                break;
        }
    }
    #ifdef DEBUG
    else {
      Serial.println("radio not available");
    }
    #endif

    if (activated) {
      #ifdef DEBUG
        Serial.println("activated");
      #endif

        orientation.update(deltaTime);
        motorController.calculatePower(orientation.velocity.z, orientation.angles.x, orientation.angles.y, deltaTime);
        #ifndef DEBUGG
          analogWrite(MOTOR_TL_Pin, 128 + (motorPowerTL / 2));   
          analogWrite(MOTOR_TR_Pin, 128 + (motorPowerTR / 2));   
          analogWrite(MOTOR_BR_Pin, 128 + (motorPowerBR / 2));   
          analogWrite(MOTOR_BL_Pin, 128 + (motorPowerBL / 2));
        #endif
    }
    else {
        #ifdef DEBUG
          Serial.println("not activated");
        #endif

        if (millis() % 1000 == 0) {
            radioLog("Waiting for activation", true);
        }
    }

    // Output
    if (millis() % 20 == 0) sendRadio();

    sequenceTelemetry();
}

void setDeltaTime() {
    unsigned long currentTime = micros();
    deltaTime = (float)(currentTime - previousTime) / 1000000;
    previousTime = currentTime;
    if (deltaTime == 0) {
        deltaTime = 0.000001;
    }
}

void sendRadio() {
    #ifdef DEBUG
      Serial.print("(before) SendStack count: ");
      Serial.println(sendStack.count);
    #endif

    radio.stopListening();
    while (sendStack.count > 0) {
        #ifdef DEBUG 
          Serial.println("attempting to send");
        #endif

        RadioMessage message = sendStack.pop();
        bool result = radio.write(&message, sizeof(message));

        #ifdef DEBUG
          printRadioMessage(message);
        #endif

        if (!result){
            #ifdef DEBUG
              Serial.println("failed to send");
            #endif

            sendStack.push(message);
            break;
        }

        #ifdef DEBUG
          Serial.println("successfully sent");
        #endif
    }
    radio.startListening();
    #ifdef DEBUG
      Serial.print("(after) SendStack count: ");
      Serial.println(sendStack.count);
    #endif
}

void radioLog(const char* message, bool important) {
    #define DRONE_LOG
    RadioMessage logMessage;
    logMessage.messageType = _MSG_DRONE_LOG;
    uint8_t messageLength = strlen(message);
    messageLength = (messageLength < 31) ? messageLength : 31;
    memcpy(logMessage.dataBuffer, message, messageLength);
    
    if (important)
        sendStack.push(logMessage);
    else
        sendStack.queue(logMessage);
}

#ifdef DEBUG
void printRadioMessage(RadioMessage message) {
    Serial.print("Message type: ");
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

void activate() {
  if (activated) break;
  activated = true;
  orientation.begin(500);
  #ifndef DEBUG
  // Ramp up motors to 50%
  power = 0;
  time = 0;
  while (time < 1) {
      setDeltaTime();
      analogWrite(MOTOR_TL_Pin, power);
      analogWrite(MOTOR_TR_Pin, power);
      analogWrite(MOTOR_BR_Pin, power);
      analogWrite(MOTOR_BL_Pin, power);
      time += deltaTime;
      power = 128 * time;
  }
  #endif

  radioLog("Activation complete", true);
}

void deactivate() {
  if (!activated) break;
  activated = false;
  orientation.end();
  #ifndef DEBUG
  // Ramp down motors to 0%
  power = 0;
  time = 0;
  while (time < 1) {
      setDeltaTime();
      analogWrite(MOTOR_TL_Pin, power);
      analogWrite(MOTOR_TR_Pin, power);
      analogWrite(MOTOR_BR_Pin, power);
      analogWrite(MOTOR_BL_Pin, power);
      time += deltaTime;
      power = 128 * (1 - time);
  }
  #endif
  digitalWrite(MOTOR_TL_Pin, LOW);
  digitalWrite(MOTOR_TR_Pin, LOW);
  digitalWrite(MOTOR_BR_Pin, LOW);
  digitalWrite(MOTOR_BL_Pin, LOW);
  radioLog("Deactivated", true);
}

void sequenceTelemetry() {
  if (sendStack.count <= 0) {
    messageOut.messageType = _MSG_DRONE_ACCELERATION;
    memcpy(messageOut.dataBuffer, &orientation.adjustedAcceleration, sizeof(orientation.acceleration));
    sendStack.push(messageOut);

    messageOut.messageType = _MSG_DRONE_VELOCITY;
    memcpy(messageOut.dataBuffer, &orientation.velocity, sizeof(orientation.velocity));
    sendStack.push(messageOut);

    messageOut.messageType = _MSG_DRONE_ANGULAR_VELOCITY;
    memcpy(messageOut.dataBuffer, &orientation.angularVelocity, sizeof(orientation.angularVelocity));
    sendStack.push(messageOut);

    messageOut.messageType = _MSG_DRONE_ANGLES;
    memcpy(messageOut.dataBuffer, &orientation.angles, sizeof(orientation.angles));
    sendStack.push(messageOut);

    messageOut.messageType = _MSG_DRONE_DELTATIME;
    memcpy(messageOut.dataBuffer, &deltaTime, sizeof(deltaTime));
    sendStack.push(messageOut);
  }
}