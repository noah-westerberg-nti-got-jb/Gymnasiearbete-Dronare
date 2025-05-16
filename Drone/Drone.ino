#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <MotorController.h>
#include <Vectors.h>
#include <Orientation.h>
#include <RadioSendStack.h>
#include <CountdownTimer.h>
#include <Servo.h>
#include <Vectors.h>

#define DEBUG

#define CE_PIN 7
#define CSN_PIN 6

// Radio
RF24 radio(CE_PIN, CSN_PIN, 4000000);
RadioSendStack sendStack;
RadioMessage messageIn, messageOut;

// Motors
#define MOTOR_TL_Pin 5
#define MOTOR_TR_Pin 4
#define MOTOR_BR_Pin 3
#define MOTOR_BL_Pin 2
int8_t motorPowerTL = 0, motorPowerTR = 0, motorPowerBR = 0, motorPowerBL = 0;
MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);
Servo motorTL, motorTR, motorBR, motorBL;

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
const vector3<float> positionOffset = {0, 0.075, 0.015};
Orientation orientation(MPU, positionOffset);

bool activated = false;

bool sending = false;

CountdownTimer miscTimer;

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
    while(!configureRadio(radio)) {
      #ifdef DEBUG
        Serial.println("radio config issue");
      #endif
    }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);
    radio.startListening();

    // Set up motor controller
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(PID_Velocity);
    motorController.setPitchConstants(PID_Pitch);
    motorController.setRollConstants(PID_Roll);

    // Set up motors
    motorTL.attach(MOTOR_TL_Pin, 1000, 2000);
    motorTR.attach(MOTOR_TR_Pin, 1000, 2000);
    motorBR.attach(MOTOR_BR_Pin, 1000, 2000);
    motorBL.attach(MOTOR_BL_Pin, 1000, 2000);
    
    // Set initial position
    motorTL.writeMicroseconds(1000);
    motorTR.writeMicroseconds(1000);
    motorBR.writeMicroseconds(1000);
    motorBL.writeMicroseconds(1000);
    
    // Initial time
    previousTime = micros();
    
    radioLogPush("Connected");

    #ifdef DEBUG
      Serial.println("completed setup");
    #endif
}

void loop() {
    setDeltaTime();
    
    // Radio read
    if (radio.available()) {
        #ifdef DEBUG
          Serial.println("radio available");
        #endif
      
        sending = true;
        
        radio.read(&messageIn, sizeof(messageIn));

        #ifdef DEBUG
          printRadioMessage(messageIn);
        #endif

        switch (messageIn.messageType) {
            case _MSG_CONTROLLER_INPUT:
                ControllerInstructions controller;
                memcpy(&controller, messageIn.dataBuffer, sizeof(controller));
                
                #ifdef DEBUG
                  Serial.print("x:");
                  Serial.print((float)controller.x / 127);
                  Serial.print(" y:");
                  Serial.print((float)controller.y / 127);
                  Serial.print(" power:");
                  Serial.println((float)controller.power / 127);
                #endif

                targetVelocity = maxVelocity * (float)controller.power / 127;
                targetPitch = maxPitch * ((float)controller.x / 127);
                targetRoll = maxRoll * ((float)controller.y / 127);
                break;
            case _MSG_ACTIVATE:
                activate();
                break;
            case _MSG_DEACTIVATE:
                deactivate();
                break;
            case _MSG_SET_PID_V:
                memcpy(&PID_Velocity, messageIn.dataBuffer, sizeof(PID_Velocity));

                #ifdef DEBUG
                  Serial.print("PID-V: p:");
                  Serial.print(PID_Velocity.p);
                  Serial.print(" i:");
                  Serial.print(PID_Velocity.i);
                  Serial.print(" d:");
                  Serial.println(PID_Velocity.d);
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
                TargetRangeInstructions targetRangesIn;
                memcpy(&targetRangesIn, messageIn.dataBuffer, sizeof(targetRangesIn));
                maxVelocity = targetRangesIn.verticalVelocityMax;
                maxPitch = targetRangesIn.pitchMax;
                maxRoll = targetRangesIn.rollMax;
                break;
            case _MSG_REQUEST_TARGET_RANGES:
                TargetRangeInstructions targetRangesOut;
                targetRangesOut = {maxPitch, maxRoll, maxVelocity};
                memcpy(messageOut.dataBuffer, &targetRangesOut, sizeof(targetRangesOut));
                messageOut.messageType = _MSG_SET_TARGET_RANGES;
                sendStack.push(messageOut);
                break;
            default:
                sending = false;
                radioLogQueue("Error interpreting messageType");
                break;
        }
    }
    #ifdef DEBUG
    else {
        if (!sending)
          Serial.println("radio not available");
        else
          Serial.println("send-countdown done");
    }
    #endif

    if (activated) {
      #ifdef DEBUG
        Serial.println("activated");
      #endif

        orientation.update(deltaTime);
        motorController.calculatePower(orientation.velocity.z, orientation.angles.x, orientation.angles.y, deltaTime);

        #ifndef DEBUG
          motorTL.writeMicroseconds(map(motorPowerTL, -127, 127, 1000, 2000));
          motorTR.writeMicroseconds(map(motorPowerTR, -127, 127, 1000, 2000));
          motorBR.writeMicroseconds(map(motorPowerBR, -127, 127, 1000, 2000));
          motorBL.writeMicroseconds(map(motorPowerBL, -127, 127, 1000, 2000));
        #endif
    }
    else {
        #ifdef DEBUG
          Serial.println("not activated");
        #endif

        if (miscTimer.finished(1000)) {
              radioLogPush("Waiting for activation");
        }
        else miscTimer.start(1000);
    }

    // Output
    if (sending) 
      sending = !sendRadio();
    
    sequenceTelemetry();
}

void setDeltaTime() {
    unsigned long currentTime = micros();
    if (currentTime < previousTime) {
        deltaTime = (float)(currentTime + (0xFFFFFFFF - previousTime)) / 1000000;
    } else {
        deltaTime = (float)(currentTime - previousTime) / 1000000;
    }
    previousTime = currentTime;
    if (deltaTime == 0) {
        deltaTime = 0.000001;
    }
}

bool sendRadio() {
    #ifdef DEBUG
      Serial.print("(before) SendStack count: ");
      Serial.println(sendStack.getCount());
    #endif

    if (sendStack.getCount() <= 0)
      return true;

    radio.stopListening();
    RadioMessage message = sendStack.pop();
    bool result = radio.write(&message, sizeof(message));

    #ifdef DEBUG
      printRadioMessage(message);
    #endif

    if (!result) {
        #ifdef DEBUG
          Serial.println("failed to send");
        #endif

        sendStack.push(message);
    }
    else {    
      #ifdef DEBUG
        Serial.println("successfully sent");
      #endif
    }
    
    radio.startListening();

    return result;
}

void radioLogPush(const char* message) {
    RadioMessage logMessage;
    logMessage.messageType = _MSG_DRONE_LOG;
    uint8_t messageLength = strlen(message);
    messageLength = (messageLength < 31) ? messageLength : 31;
    memcpy(logMessage.dataBuffer, message, messageLength);
    sendStack.push(logMessage);
}

void radioLogQueue(const char* message) {
  RadioMessage logMessage;
  logMessage.messageType = _MSG_DRONE_LOG;
  uint8_t messageLength = strlen(message);
  messageLength = (messageLength < 31) ? messageLength : 31;
  memcpy(logMessage.dataBuffer, message, messageLength);
  sendStack.push(logMessage);
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
  if (activated) return;
  activated = true;
  orientation.begin(500);
  #ifndef DEBUG
  // Ramp up motors to middle position
  uint16_t power = 1000;
  float time = 0;
  while (time < 1) {
      setDeltaTime();
      motorTL.writeMicroseconds(power);
      motorTR.writeMicroseconds(power);
      motorBR.writeMicroseconds(power);
      motorBL.writeMicroseconds(power);
      time += deltaTime;
      power = 1000 + (time * 500); 
  }
  #endif

  radioLogPush("Activation complete");
}

void deactivate() {
  if (!activated) return;
  activated = false;
  orientation.end();
  #ifndef DEBUG
  // Ramp down motors to off
  uint16_t power = 1500;
  float time = 0;
  while (time < 1) {
      setDeltaTime();
      motorTL.writeMicroseconds(power);
      motorTR.writeMicroseconds(power);
      motorBR.writeMicroseconds(power);
      motorBL.writeMicroseconds(power);
      time += deltaTime;
      power = 1500 - (time * 500);
  }
  #endif
  motorTL.writeMicroseconds(1000);
  motorTR.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
  radioLogPush("Deactivated");
}

template<typename T>
void sequenceVector(vector3<T> vector, uint8_t messageType) {
  messageOut.messageType = messageType;
  float x = float(vector.x);
  float y = float(vector.y);
  float z = float(vector.z);

  memcpy(messageOut.dataBuffer + sizeof(float) * 0, &x, sizeof(float));
  memcpy(messageOut.dataBuffer + sizeof(float) * 1, &y, sizeof(float));
  memcpy(messageOut.dataBuffer + sizeof(float) * 2, &z, sizeof(float));
  sendStack.push(messageOut);
}

void sequenceTelemetry() {
  if (sendStack.getCount() > 0)
    return;

  sequenceVector(orientation.adjustedAcceleration, _MSG_DRONE_ACCELERATION);
  sequenceVector(orientation.velocity, _MSG_DRONE_VELOCITY);
  sequenceVector(orientation.rawAngularVelocity, _MSG_DRONE_ANGULAR_VELOCITY);
  sequenceVector(orientation.angles, _MSG_DRONE_ANGLES);

  messageOut.messageType = _MSG_DRONE_DELTATIME;
  memcpy(messageOut.dataBuffer, &deltaTime, sizeof(deltaTime));
  sendStack.push(messageOut);

  messageOut.messageType = _MSG_DRONE_MOTOR_POWERS;
  MotorPowers motorPowers = {motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL};
  memcpy(messageOut.dataBuffer, &motorPowers, sizeof(motorPowers));
  sendStack.push(messageOut);

  messageOut.messageType = (activated) ? _MSG_DEACTIVATE : _MSG_ACTIVATE;
  sendStack.push(messageOut);
}