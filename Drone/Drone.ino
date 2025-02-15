#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <MotorControl.h>
#include <Vectors.h>
#include <Orientation.h>
#include <RadioSendStack.h>

#define DEBUG
#ifdef DEBUG
char* testSwitch = "";
#endif

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
float velocityP = 0; 
float velocityI = 0; 
float velocityD = 0;
float maxVelocity;

float targetPitch;
float pitchP = 0; 
float pitchI = 0; 
float pitchD = 0;
float maxPitch;

float targetRoll;
float rollP = 0; 
float rollI = 0; 
float rollD = 0;
float maxRoll;

PID_Instructions pidIn, pidOut;

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
      Serial.begin(9600);
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

    // Set up motor controller
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(&velocityP, &velocityI, &velocityD);
    motorController.setPitchConstants(&pitchP, &pitchI, &pitchD);
    motorController.setRollConstants(&rollP, &rollI, &rollD);

    pinMode(MOTOR_TL_Pin, OUTPUT);
    pinMode(MOTOR_TR_Pin, OUTPUT);
    pinMode(MOTOR_BR_Pin, OUTPUT);
    pinMode(MOTOR_BL_Pin, OUTPUT);
    
    // Initial time
    previousTime = millis();

    consoleLog("Connected", true);

    #ifdef DEBUG
      Serial.println("completed setup");
    #endif
}

void loop() {
    #ifdef DEBUG
      Serial.println("loop");
    #endif

    setDeltaTime();
    
    // Radio read
    if (radio.available()) {
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
          Serial.print("message type:");
          Serial.println(messageIn.messageType);
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
                  Serial.print((float)controller.power / 127);
                #endif

                targetVelocity = maxVelocity * (float)controller.power / 127;
                targetPitch = maxPitch * ((float)controller.stick_X / 127);
                targetRoll = maxRoll * ((float)controller.stick_Y / 127);
                break;
            case _MSG_ACTIVATE:
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
                
                consoleLog("Activation complete", true);
                break;
            case _MSG_DEACTIVATE:
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

                consoleLog("Deactivated", true);
                break;
            case _MSG_SET_PID_V:
                #ifdef DEBUG
                  testSwitch = "test";
                #endif

                memcpy(&pidIn, messageIn.dataBuffer, sizeof(pidIn));
                velocityP = pidIn.k_p;
                velocityI = pidIn.k_i;
                velocityD = pidIn.k_d;
                break;
            case _MSG_SET_PID_P:
                memcpy(&pidIn, messageIn.dataBuffer, sizeof(pidIn));
                pitchP = pidIn.k_p;
                pitchI = pidIn.k_i;
                pitchD = pidIn.k_d;
                break;
            case _MSG_SET_PID_R:
                memcpy(&pidIn, messageIn.dataBuffer, sizeof(pidIn));
                rollP = pidIn.k_p;
                rollI = pidIn.k_i;
                rollD = pidIn.k_d;
                break;
            case _MSG_REQUEST_PID_V:
                pidOut.k_p = velocityP;
                pidOut.k_i = velocityI;
                pidOut.k_d = velocityD;
                memcpy(messageOut.dataBuffer, &pidOut, sizeof(pidOut));
                sendStack.push(messageOut);
                break;
            case _MSG_REQUEST_PID_P:
                pidOut.k_p = pitchP;
                pidOut.k_i = pitchI;
                pidOut.k_d = pitchD;
                memcpy(messageOut.dataBuffer, &pidOut, sizeof(pidOut));
                sendStack.push(messageOut);
                break;
            case _MSG_REQUEST_PID_R:
                pidOut.k_p = rollP;
                pidOut.k_i = rollI;
                pidOut.k_d = rollD;
                memcpy(messageOut.dataBuffer, &pidOut, sizeof(pidOut));
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
                sendStack.push(messageOut);
                break;
            default:
                consoleLog("Error interpreting messageType", false);
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
        analogWrite(MOTOR_TL_Pin, 128 + (motorPowerTL / 2));   
        analogWrite(MOTOR_TR_Pin, 128 + (motorPowerTR / 2));   
        analogWrite(MOTOR_BR_Pin, 128 + (motorPowerBR / 2));   
        analogWrite(MOTOR_BL_Pin, 128 + (motorPowerBL / 2));
    }
    else {
        #ifdef DEBUG
        Serial.println("not activated");
        #endif

        if (millis() % 5000 == 0) {
            consoleLog("Waiting for activation", true);
        }
    }

    // Output
    sendRadio();

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

    #ifdef DEBUG
      Serial.println(testSwitch);
      delay(200);
    #endif
}

void setDeltaTime() {
    unsigned long currentTime = millis();
    deltaTime = (float)(currentTime - previousTime) / 1000;
    previousTime = currentTime;
}

void sendRadio() {
    while (sendStack.count > 0 && !radio.available()) {
        #ifdef DEBUG
          Serial.println("attempting to send");
        #endif

        RadioMessage message = sendStack.pop();
        bool result = radio.write(&message, sizeof(message));
        radio.startListening();

        #ifdef DEBUG
          Serial.print("Message: ");
          for (int i = 0; i < 32; i++) {
            Serial.print((int)message.dataBuffer[i]);
            Serial.print(" ");
          }
          Serial.print("Length: ");
          Serial.println((int)sizeof(message));
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
}

void consoleLog(const char* message, bool important) {
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