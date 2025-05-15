# Remote Controlled Drone

This repository contains the software part of a multi-person project with the goal of creating a remote-controlled drone using Arduino microcontrollers, basic components, and 3D printers.

## Table of Contents

- [Remote Controlled Drone](#remote-controlled-drone)
  - [Table of Contents](#table-of-contents)
  - [Project Abstract](#project-abstract)
  - [Materials](#materials)
  - [Drone](#drone)
    - [Basic Structure](#basic-structure)
      - [Setup](#setup)
      - [Main Loop](#main-loop)
        - [Radio Input](#radio-input)
        - [Flight Control](#flight-control)
        - [Radio Output](#radio-output)
      - [Important Helper Functions](#important-helper-functions)
        - [`setDeltaTime`](#setdeltatime)
        - [`sendRadio`](#sendradio)
        - [Activation and Deactivation Functions](#activation-and-deactivation-functions)
    - [Important Classes](#important-classes)
      - [`PID`](#pid)
      - [`MotorController`](#motorcontroller)
      - [`Orientation`](#orientation)
      - [`RadioSendStack`](#radiosendstack)
      - [`Timer` (drone)](#timer-drone)
      - [`vector3<T>`](#vector3t)
        - [Other](#other)
    - [Summary of Drone](#summary-of-drone)
  - [Receiver](#receiver)
    - [Basic Structure](#basic-structure-1)
      - [Setup](#setup-1)
      - [Main Loop](#main-loop-1)
        - [Serial Input](#serial-input)
        - [Radio Input](#radio-input-1)
    - [Important Classes](#important-classes-1)
      - [`InstructionHandler`](#instructionhandler)
      - [`Timer` (receiver)](#timer-receiver)
    - [Summary of Receiver](#summary-of-receiver)
  - [Control Panel](#control-panel)
    - [Basic Structure](#basic-structure-2)
      - [Setup](#setup-2)
      - [Main Loop](#main-loop-2)
        - [User Input](#user-input)
        - [Serial Communication](#serial-communication)
        - [Connection Status Indicators](#connection-status-indicators)
        - [Data Collection](#data-collection)
    - [Important Classes](#important-classes-2)
      - [`SerialMessage`](#serialmessage)
      - [`SerialReader`](#serialreader)
      - [`Terminal`](#terminal)
      - [`WritingHandler`](#writinghandler)
    - [Summary of Control Panel](#summary-of-control-panel)
  - [Simulation](#simulation)
    - [Features](#features)
    - [How to Use](#how-to-use)
    - [Notes](#notes)
    - [Summary of Simulation](#summary-of-simulation)
  - [Summary](#summary)

## Project Abstract

....

## Materials

- 4 DarwinFPV 1104 motors
- 4 Gemfan 3016 propellers
- 2 Ebyte E01-ML01DP5 radio tranceivers
- 1 MPU 6050 gyroscope/accelerometer
- 1 Capacitor
- 1 7.4 Volt 3000 mAh LiPo 2S battery
- 1 3.7 Volt 300 mAh LiPo 1S battery
- 1 Jhemcu EM-40A 4 in 1 ESC
- 1 Arduino MKR Zero
- 1 Arduino Uno
- 3D-printer + filament
- Soldering and wire handling equipment

## Drone

[`Drone.ino`](Drone/Drone.ino) is intended to be used with an `Arduino MKR Zero` alongside an [`MPU 6050`](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/) gyroscope/accelerometer, an [`nRF24L01`](https://github.com/nRF24/RF24) radio device, and a set of `DarwinFPV 1104` brushless motors.

> **_NOTE:_** It will likely work with other components, with some minor modifications to the code.

### Basic Structure

[`Drone.ino`](Drone/Drone.ino) is the sketch file for the drone and, by using both public `Arduino` libraries and the custom-made [`DroneLibrary`](DroneLibrary), it handles the control flow for the drone.

> **_NOTE:_** To use `DroneLibrary` with the Arduino IDE, copy the `DroneLibrary` folder into your `Arduino/libraries` directory.

#### Setup

As with most `Arduino` sketches, the [`Drone.ino`](Drone/Drone.ino) file includes a setup function.

```cpp
void setup() {
    ...

    while (!radio.begin()) { }
    while (!configureRadio(radio)) { }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);
    radio.startListening();

    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(PID_Velocity);
    motorController.setPitchConstants(PID_Pitch);
    motorController.setRollConstants(PID_Roll);

    motorTL.attach(MOTOR_TL_Pin, 1000, 2000);
    motorTR.attach(MOTOR_TR_Pin, 1000, 2000);
    motorBR.attach(MOTOR_BR_Pin, 1000, 2000);
    motorBL.attach(MOTOR_BL_Pin, 1000, 2000);

    motorTL.writeMicroseconds(1000);
    motorTR.writeMicroseconds(1000);
    motorBR.writeMicroseconds(1000);
    motorBL.writeMicroseconds(1000);

    previousTime = micros();

    radioLogPush("Connected");

    ...
}
```

Firstly, the radio transceiver is configured:
- `radio.begin()` initializes the radio transceiver.
- `configureRadio(radio)` configures the transceiver's settings.
- Writing and reading pipes are opened with addresses defined in [`RadioData.h`](DroneLibrary/RadioData.h).
- `radio.startListening()` enables the transceiver to listen for instructions.

Secondly, the motor controller and motors are configured:
- The `MotorController` is set up with target values and PID constants.
- The motors are attached to their respective pins and initialized to their off position (1000 microseconds).

Lastly, the initial time is recorded, and a connection message is queued for transmission.

#### Main Loop

The main control flow is found within the `loop` function and can be divided into three sections: radio input, flight control, and radio output.

##### Radio Input

```cpp
void loop() {
    setDeltaTime();

    if (radio.available()) {
        radio.read(&messageIn, sizeof(messageIn));
        switch (messageIn.messageType) {
            case _MSG_CONTROLLER_INPUT:
                // code...
            case _MSG_ACTIVATE:
                // code...
            case _MSG_SET_PID_P:
                // code...
        }
    }

    ...
}
```

This part of the code checks for incoming radio messages and processes them based on their type. It handles controller input, activation/deactivation commands, PID updates, and target range updates.

##### Flight Control

```cpp
void loop() {
    ...

    if (activated) {
        orientation.update(deltaTime);
        motorController.calculatePower(orientation.velocity.z, orientation.angles.x, orientation.angles.y, deltaTime);

        motorTL.writeMicroseconds(map(motorPowerTL, -127, 127, 1000, 2000));
        motorTR.writeMicroseconds(map(motorPowerTR, -127, 127, 1000, 2000));
        motorBR.writeMicroseconds(map(motorPowerBR, -127, 127, 1000, 2000));
        motorBL.writeMicroseconds(map(motorPowerBL, -127, 127, 1000, 2000));
    } else {
        if (miscTimer.finished(1000)) {
            radioLogPush("Waiting for activation");
        } else {
            miscTimer.start(1000);
        }
    }

    ...
}
```
This part of the code updates the drone's sensor readings and calculates the required power for each motor if the drone is activated. 
- Firstly, the `update` method from the [Orientation](DroneLibrary/Orientation.h) class is called to collect and process data from the drone's gyroscope.
- Secondly, the new values are passed into the [`MotorController`](#motorcontroller) class' `calculatePower` method to calculate the optimal motor powers for the current moment.
- Lastly, a signal with the desired power is sent to each of the motors using `writeMicroseconds`.

If the drone isn't activated, it periodically sends a message to the controller which displays that it is connected and ready to be activated.

##### Radio Output

```cpp
void loop() {
    ...

    if (sending) {
        sending = !sendRadio();
    }

    sequenceTelemetry();

    ...
}
```

This part of the code handles outgoing radio messages. It sends queued messages and sequences telemetry data for transmission.

#### Important Helper Functions

##### `setDeltaTime`

```cpp
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
```

Calculates the time delta between the current and previous loop iterations.

##### `sendRadio`

```cpp
bool sendRadio() {
    if (sendStack.getCount() <= 0) return true;

    radio.stopListening();
    RadioMessage message = sendStack.pop();
    bool result = radio.write(&message, sizeof(message));
    if (!result) {
        sendStack.push(message);
    }
    radio.startListening();
    return result;
}
```

Sends queued radio messages and retries if a message fails to send.

##### Activation and Deactivation Functions

```cpp
void activate() {
    if (activated) return;
    activated = true;
    orientation.begin(500);
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
    radioLogPush("Activation complete");
}

void deactivate() {
  if (!activated) return;
  activated = false;
  orientation.end();
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
  motorTL.writeMicroseconds(1000);
  motorTR.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
}
```

The activation and deactivation functions start by calling the begin and end methods on the [`Orientation`](DroneLibrary/Orientation.h) class respectively, and set an activation flag to be either true or false respectively. After, they proceed to either ramp up or down the motors to or from 50% respectively. They end by queuing a message to be sent by radio, informing the user that the drone is either activated or deactivated. Additionally, the deactivation function also ensures that all the motor pins are turned off completely.

### Important Classes

For the drone to function properly, multiple classes were created to handle critical tasks.

#### `PID`

The [`PID`](DroneLibrary/PIDController.h) class implements a standard PID controller with a scalar value for each of the current **proportional** error, the cumulative **integral** of the error, and the current **derivative** of the error.

**To use the class:**
1. First, create an object. 
   ```cpp
    PID pid;
   ```
2. Then, use the `setConstants` method to set the PID constants. You can either:
   - Pass pointers to the scalar values:
     ```cpp
     float p = ..., i = ..., d = ...;
     pid.setConstants(&p, &i, &d);
     ```
   - Or pass a `PID_Instructions` struct:
     ```cpp
     PID_Instructions pidValues = {1.0, 0.5, 0.1};
     pid.setConstants(pidValues);
     ```
3. Use the `setTarget` method to set a pointer to the PID controller's target value.
   ```cpp
   float target = ...;
   pid.setTarget(&target);
   ```
4. Lastly, to acquire the controller's output, use the `calculate` method.
   ```cpp
   float output = pid.calculate(inputValue, deltaTime);
   ```

#### `MotorController`

The [`MotorController`](DroneLibrary/MotorController.h) class combines multiple [`PID`](#pid) controllers to handle the pitch, roll, and vertical velocity of the drone.

**To use the class:**
1. First, create an object by passing references to the motor power variables.
   ```cpp
    int8_t motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL;
    MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);
   ```
2. Then, use the `setTargetValues` method to set pointers to the target values for velocity, pitch, and roll.
   ```cpp
    float targetVelocity = ..., targetPitch = ..., targetRoll = ...;
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
   ```
3. Use the `setVelocityConstants`, `setPitchConstants`, and `setRollConstants` methods to set the PID constants for each respective controller.
   ```cpp
    PID_Instructions velocityConstants = ..., pitchConstants = ..., rollConstants = ...;
    motorController.setVelocityConstants(velocityConstants);
    motorController.setPitchConstants(pitchConstants);
    motorController.setRollConstants(rollConstants);
   ```
4. Lastly, to calculate the motor power values, use the `calculatePower` method by passing the current velocity, pitch, roll, and delta time.
   ```cpp
    motorController.calculatePower(currentVelocity, currentPitch, currentRoll, deltaTime);
   ```

#### `Orientation`

The [`Orientation`](DroneLibrary/Orientation.h) class is used to collect data from a connected [`MPU60X0`](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/) gyroscope-accelerometer and process it into usable acceleration, velocity, and Euler-angle vectors.

**To use the class:**
1. First, create an object.
   ```cpp
   Orientation orientation(MPUAddress, positionOffset);
   ```
   - `MPUAddress`: The I2C address of the MPU.
   - `positionOffset`: The sensor's offset from the center of gravity.

2. **Initialize the MPU:**
   ```cpp
   orientation.begin();
   ```
   - Optionally, pass the number of cycles for offset calculation:
     ```cpp
     orientation.begin(500);
     ```

3. **Update orientation data:**
   ```cpp
   orientation.update(deltaTime);
   ```

4. **Access processed data:**
   - `rawAngularVelocity`: Raw angular velocity from the gyroscope (**°/s**).
   - `angles`: Euler angles (**°**).
   - `rawAcceleration`: Raw acceleration from the accelerometer (**m/s²**).
   - `acceleration`: Corrected acceleration at the center of gravity (**m/s²**).
   - `adjustedAcceleration`: Acceleration adjusted to the world frame (**m/s²**).
   - `velocity`: Velocity calculated from the adjusted acceleration (**m/s**).

5. **End the session:**
   ```cpp
   orientation.end();
   ```

The `Orientation` class provides robust tools for processing sensor data, making it ideal for applications requiring precise motion tracking and orientation correction.

#### `RadioSendStack`

The [`RadioSendStack`](DroneLibrary/RadioSendStack.h) class is used to handle a list of messages to be sent by radio and does this by implementing a linked list structure.

**To use the class:**
1. First, create an object.
   ```cpp
    RadioSendStack sendStack;
   ```
2. Then, use the `push` or `queue` methods to add a new message to the front or back of the list, respectively.
   ```cpp
    RadioMessage message = ...;
    sendStack.push(message);
    // or
    sendStack.queue(message);
   ```
3. Use the `peek` and `pop` methods to retrieve messages from the list:
   - `peek` retrieves a message without removing it. It can be used without arguments to retrieve the first element or with an index to retrieve a specific element.
     ```cpp
     RadioMessage message = sendStack.peek();
     ```
   - `pop` retrieves and removes a message from the list. It can be used without arguments to collect the first element or with an index to collect a specific element.
     ```cpp
     RadioMessage message = sendStack.pop();
     ```
4. Use the `getCount` method to retrieve the total number of messages in the list.
   ```cpp
    uint8_t count = sendStack.getCount();
   ```
5. Use the `clear` method to remove all elements from the list.
   ```cpp
    sendStack.clear();
   ```

#### `Timer` (drone)

The [`Timer`](DroneLibrary/Timer.h) class is a utility for managing timed events. It allows you to start, stop, and check if a timer has finished. It is particularly useful for scheduling periodic tasks or delays.

**To use the class:**
1. Create a `Timer` object.
   ```cpp
   Timer timer;
   ```
2. Use the `start` method to initialize the timer with a duration (in milliseconds).
   ```cpp
   timer.start(1000); // 1 second
   ```
3. Use the `finished` method to check if the timer has completed.
   ```cpp
   if (timer.finished()) {
       // Timer has finished
   }
   ```
4. Optionally, use the overloaded `finished` method to restart the timer with a new duration if it has finished.
   ```cpp
   if (timer.finished(500)) {
       // Timer has finished and restarted with a 500ms duration
   }
   ```
5. Use the `stop` method to reset the timer.
   ```cpp
   timer.stop();
   ```

#### `vector3<T>`

The [`vector3<T>`](DroneLibrary/Vectors.h) struct is a 3-dimensional vector template for simplifying vector calculations in tasks like physics simulations and orientation tracking.

**Features:**
- **Basic Operations**: Addition, subtraction, scalar multiplication, and division.
- **Dot Product**: Calculates the scalar product of two vectors.
- **Cross Product**: Calculates the vector product of two vectors.
- **Magnitude**: Computes the vector's length.
- **Projection**: Projects one vector onto another.

**To use the class:**
1. Create a vector:
   ```cpp
   vector3<float> v1 = {1.0, 2.0, 3.0};
   ```
2. Perform operations:
   ```cpp
   vector3<float> sum = v1 + v2; // Addition
   float dot = v1 * v2;          // Dot product
   vector3<float> cross = crossProduct(v1, v2); // Cross product
   ```

**Public Members:**
- `T x, y, z`: The vector components.

**Public Methods:**
- `float magnitude()`: Returns the vector's length.

The `vector3<T>` struct is a versatile tool for 3D vector calculations, ideal for physics, robotics, and drone orientation systems.

##### Other

There are also a few structs defined in [`RadioData.h`](DroneLibrary/RadioData.h) to simplify data handling:

- `ControllerInstructions` for handling controller input
```cpp
struct ControllerInstructions {
    int8_t x;
    int8_t y;
    int8_t power;
};
```

- `PID_Instructions` for handling PID settings
```cpp
struct PID_Instructions {
    float k_p, k_i, k_d;
};
```

- `TargetRangeInstructions` for handling the different ranges that the drone's target values can reside in
```cpp
struct TargetRangeInstructions {
    float pitchMax, rollMax, verticalVelocityMax;
};
```

- `RadioMessage` for storing a message to be sent by radio with its associated `messageType`
```cpp
struct RadioMessage {
    uint8_t messageType;
    uint8_t dataBuffer[31];
};
```

### Summary of [Drone](#drone)

The `Drone.ino` sketch is designed for an Arduino MKR Zero and integrates with various components like the [`MPU 6050`](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/), [`nRF24L01`](https://github.com/nRF24/RF24) radio device, and DarwinFPV 1104 brushless motors. The setup function initializes the radio transceiver and motor configurations, while the main loop handles radio input, flight control, and radio output. Important helper functions include `setDeltaTime` for time calculations and `sendRadio` for managing radio communications. The drone's functionality is supported by several key classes such as `PID`, `MotorController`, `Orientation`, and `RadioSendStack`, which manage tasks like PID control, motor power calculations, orientation data processing, and radio message handling. Custom data types like `vector3<T>` are also used to facilitate data management.

## Receiver

[`Receiver.ino`](Receiver/Receiver.ino) is intended to be used with an `Arduino UNO` alongside an [`nRF24L01`](https://github.com/nRF24/RF24) radio device, connected by USB to a computer that is running the [`Control Panel`](#control-panel) program.

### Basic Structure

[`Receiver.ino`](Receiver/Receiver.ino) is the sketch file for the receiver and, by using both public `Arduino` libraries and the custom-made [`DroneLibrary`](DroneLibrary), it handles the control flow for relaying messages between the drone and the controller.

#### Setup

As with most `Arduino` sketches, the [`Receiver.ino`](Receiver/Receiver.ino) file includes a setup function.

```cpp
void setup() {
    ...

    Serial.begin(115200);
    while (!Serial);

    if (!radio.begin()) {
        Serial.println(F("Radio hardware not responding!"));
        while (true);
    }
    if (!configureRadio(radio)) {
        Serial.println("Radio configuration failed");
        while (true);
    }
    radio.openWritingPipe(RECEIVER_ADDRESS);
    radio.openReadingPipe(1, DRONE_ADDRESS);
    radio.startListening();

    previousTime = micros();

    ...
}
```

Firstly, Serial is configured with a baud rate of `115200`.  
Secondly, the radio transceiver is configured:
- `radio.begin()` initializes the radio transceiver.
- `configureRadio(radio)` configures the transceiver's settings.
- Writing and reading pipes are opened with addresses defined in [`RadioData.h`](DroneLibrary/RadioData.h).
- `radio.startListening()` enables the transceiver to listen for instructions.

Lastly, the initial time is recorded.

> **_NOTE:_** The transceiver's initialization and configuration are placed in while loops to prevent the program from proceeding if any part fails.

#### Main Loop

The main control flow is found within the `loop` function and can be divided into two sections: serial input and radio input.

##### Serial Input

```cpp
void loop() {
    ...

    if (instructionHandler.read()) {
        uint8_t messageType = instructionHandler.getData(readBuffer);
        messageOut.messageType = messageType;
        memcpy(messageOut.dataBuffer, readBuffer, sizeof(messageOut.dataBuffer));

        bool result = false;
        if (sending) 
            result = send();
        if (result) {
            sending = false;
            instructionHandler.acknowledge(messageType);
        }

        ...
    }

    ...
}
```

This part of the code checks for incoming serial messages from the controller using the [`InstructionHandler`](#instructionhandler) class. If a message is received, it processes the message, sends it to the drone via the radio, and acknowledges the message if the transmission is successful.

##### Radio Input

```cpp
void loop() {
    ...

    if (radio.available()) {
        sending = true;

        connectionStatus = connectionStatus | 0b010;

        radio.read(&messageIn, sizeof(messageIn));

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
                instructionHandler.write(messageIn.dataBuffer, messageIn.messageType);
                break;
        }
    }

    instructionHandler.write(&connectionStatus, _MSG_CONNECTION_STATUS);
    instructionHandler.write((uint8_t*)&deltaTime, _MSG_RECEIVER_DELTATIME);

    ...
}
```

This part of the code checks for incoming radio messages from the drone. If a message is received, it processes the message based on its type and forwards it to the controller via the serial connection. It also updates the connection status and delta time.

### Important Classes

#### `InstructionHandler`

The [`InstructionHandler`](DroneLibrary/InstructionHandler.h) class is used to manage serial communication between the receiver and the controller. It handles reading, writing, and acknowledging messages.

**To use the class:**
1. Create an object.
   ```cpp
   InstructionHandler instructionHandler;
   ```
2. Use the `read` method to check if a new message has been received via serial.
   ```cpp
   if (instructionHandler.read()) {
       // Process the message
   }
   ```
3. Use the `getData` method to retrieve the message data and type.
   ```cpp
   uint8_t messageType = instructionHandler.getData(buffer);
   ```
4. Use the `write` method to send a message via serial.
   ```cpp
   instructionHandler.write(data, messageType);
   ```
5. Use the `acknowledge` method to send an acknowledgment for a received message.
   ```cpp
   instructionHandler.acknowledge(messageType);
   ```

#### `Timer` (receiver)

See [`Timer`](#timer-drone).

### Summary of [Receiver](#receiver)

The `Receiver.ino` sketch acts as a bridge between the drone and the controller. It uses an [`nRF24L01`](https://github.com/nRF24/RF24) radio device to communicate with the drone and a serial connection (115200 baud) to communicate with the controller. The sketch relies on the `InstructionHandler` class to manage serial communication and ensures that messages are properly relayed between the two systems. The setup function initializes the radio transceiver and serial communication, while the main loop handles serial input, radio communication, and message forwarding. The receiver monitors connection status for both the drone and its activation state and uses timer-based message handling to improve communication reliability. Additionally, it ensures acknowledgment of serial messages and processes drone logs for better debugging and monitoring.

## Control Panel

The `Control Panel` is a web-based application designed to interact with the drone via the [`Receiver`](#receiver). It provides a graphical interface for controlling the drone, adjusting PID values, and monitoring telemetry data.

### Basic Structure

The `Control Panel` application is built using HTML, CSS, and JavaScript. It handles the control flow for interacting with the drone and receiver by using both browser APIs and custom JavaScript classes.

#### Setup

The setup process involves initializing the serial connection and setting up event listeners for user interactions:
- A button allows the user to select a serial port using the browser's Web Serial API.
- Once a port is selected, it is opened with predefined settings.
- Periodic read and write operations are scheduled to handle communication with the receiver.
- A message is logged to the terminal upon successful connection.

#### Main Loop

The main control flow is divided into two sections: user input and serial communication.

##### User Input

- Handles keyboard input for controlling the joystick and power levels.
- Updates the joystick position and power level based on user input.
- Supports additional input methods such as mouse and gamepad for controlling the drone.

##### Serial Communication

- Sends instructions to the receiver via the serial port.
- Uses a dedicated class to manage outgoing messages and ensure proper communication flow.

##### Connection Status Indicators

The `Control Panel` displays the connection status of the receiver, drone, and drone activation state using color-coded indicators:

  - <span style="color:green;">connected</span>
  - <span style="color:red;">disconnected</span>
  - <span style="color:grey;">unkown</span>

These indicators update dynamically based on received messages and reset to disconnected/unkown if no updates are received within a timeout period.

##### Data Collection

The `Control Panel` application includes functionality for collecting flight data during the drone's operation. This data is periodically gathered and saved in a structured format for analysis. The data includes information such as time, acceleration, velocity, angular velocity, rotation, controller power, joystick position, FPS, and motor powers.

Once data collection stops, the collected data is saved as a `JSON` file with a timestamped filename in the `FlightData` folder.

For a detailed explanation of the flight data format and structure, refer to the [Flight Data README](FlightData/README.md).

This feature allows users to analyze the drone's performance and behavior during flights.

### Important Classes

For the `Control Panel` to function properly, multiple classes were created to handle critical tasks.

#### `SerialMessage`

The `SerialMessage` class defines the structure of messages exchanged between the `Control Panel` and the `Receiver`.

**To use the class:**
1. First, create an object.
   ```javascript
   const message = new SerialMessage();
   ```
2. Use the `set` method to initialize a message with a specific type.
   ```javascript
   message.set(type);
   ```
3. Use the `reset` method to reset the message state.
   ```javascript
   message.reset();
   ```

#### `SerialReader`

The `SerialReader` class handles reading and parsing incoming serial data.

**To use the class:**
1. First, create an object.
   ```javascript
   const reader = new SerialReader();
   ```
2. Use the `read` method to process incoming bytes.
   ```javascript
   const result = reader.read(byte);
   ```
   - **Structure of `result`:**
     - `done` (boolean): Indicates whether a complete message has been read.
     - `reading` (boolean): Indicates whether the reader is currently processing a message.
     - `data` (Uint8Array): Contains the raw data of the message (only available when `done` is `true`).
     - `messageType` (number): The numeric type of the message (only available when `done` is `true`).
     - `messageTypeName` (string): The name of the message type (only available when `done` is `true`).

     Example:
     ```javascript
     const result = serialReader.read(byte);
     if (result.done) {
         console.log(result.messageTypeName); // e.g., "pid-velocity"
         console.log(result.data); // Uint8Array containing the message data
     }
     ```
3. Use specialized methods to parse specific data types:
   - `readPIDInstruction(data)`
   - `readTargetRanges(data)`
   - `readVector(data)`
   - `readDeltaTime(data)`
   - `readMotorPowers(data)`

#### `Terminal`

The `Terminal` class manages the display of logs and messages in the terminal section of the interface.

**To use the class:**
1. First, create an object.
   ```javascript
   const terminal = new Terminal();
   ```
2. Use the `addToTerminal` method to add a log entry.
   ```javascript
   terminal.addToTerminal(log);
   ```
3. Use the `clear` method to clear the terminal.
   ```javascript
   terminal.clear();
   ```

#### `WritingHandler`

The `WritingHandler` class manages outgoing instructions to the `Receiver`.

**To use the class:**
1. First, create an object.
   ```javascript
   const handler = new WritingHandler();
   ```
2. Use the `set` or `setWithConfirm` method to set the current instructions and type.
   ```javascript
   handler.set(instructions, type);
   handler.setWithConfirm(instructions, type, value);
   ```
   - **Details:**
     - `set`: Sets the current instructions and their type. Marks the handler as having instructions to send.
       Example:
       ```javascript
       const instructions = getControllerInstructions(joystick.x, joystick.y, power);
       writingHandler.set(instructions, SerialMessage.messageTypeFromName.get("controller-instructions"));
       ```
     - `setWithConfirm`: Similar to `set`, but also sets a confirmation value to ensure the data is acknowledged before proceeding. Used when the data requires confirmation from the receiver.
       Example:
       ```javascript
       const pidValues = { p: 1.0, i: 0.5, d: 0.1 };
       const instructions = getPIDInstructions(pidValues, "velocity");
       writingHandler.setWithConfirm(instructions, SerialMessage.messageTypeFromName.get("pid-velocity"), pidValues);
       ```
3. Use the `get` method to retrieve the current instructions if available.
   ```javascript
   const instructions = handler.get();
   ```
   - **Details:**
     - Retrieves the current instructions if available.
     - If no instructions are set, it defaults to sending controller instructions.
     - Example:
       ```javascript
       const instructions = writingHandler.get();
       ```
4. Use the `acceptAcknowledge` method to handle acknowledgment messages.
   ```javascript
   handler.acceptAcknowledge(messageType);
   ```
   - **Details:**
     - Handles acknowledgment messages from the receiver.
     - If the acknowledgment matches the current instructions type, it clears the current instructions.
     - Example:
       ```javascript
       writingHandler.acceptAcknowledge(SerialMessage.messageTypeFromName.get("pid-velocity"));
       ```

### Summary of [Control Panel](#control-panel)

The `Control Panel` application provides an intuitive interface for controlling the drone and monitoring its telemetry. It uses the `SerialMessage`, `SerialReader`, `Terminal`, and `WritingHandler` classes to manage communication with the `Receiver`. The application supports joystick, keyboard, and gamepad inputs for controlling the drone and allows real-time adjustments to PID constants and target ranges. The `WritingHandler` class ensures proper acknowledgment of instructions sent to the receiver, improving communication reliability. Additionally, the application collects and saves flight data for post-flight analysis, enabling users to evaluate the drone's performance and behavior during flights.


## Simulation

The simulation is a Unity-based project designed to calibrate the PID settings for the drone. It provides a virtual environment where the drone's behavior can be tested and adjusted without requiring physical hardware. The simulation is intended to be run in the Unity editor, as the PID values and other settings can only be updated through the Inspector.

### Features

- **PID Calibration**: Adjust the proportional, integral, and derivative constants for pitch, roll, and velocity in real-time.
- **Propeller Visualization**: Displays the forces applied by each propeller using Gizmos for better understanding of the drone's behavior.
- **Center of Mass Adjustment**: Allows visualization and adjustment of the drone's center of mass to observe its impact on stability.
- **Velocity and Orientation Feedback**: Provides visual feedback on the drone's velocity and orientation using Gizmos.

### How to Use

1. Open the Unity project located in the `Simulation/ga-drone-simulation` folder.
2. Select the GameObject in the scene with the `DroneController` component.
3. Use the Inspector to adjust the PID constants (`P`, `I`, `D`) for pitch, roll, and velocity, and the drones mass and center of mass.
4. Play the scene in the Unity editor to observe the drone's behavior.
5. Use the keyboard or a game controller to provide input:
   - **Throttle**: `Left Shift` or `Left Trigger` to decrease, and `Space` or `Right Trigger` to increase.
   - **Pitch**: `W` (forward) and `S` (backward).
   - **Roll**: `A` (left) and `D` (right).
6. Observe the drone's response and adjust the PID values as needed to achieve stable flight.

### Notes

- The simulation uses a custom `DroneController` script to emulate the drone's physics and control logic.
- The `CameraController` script ensures the camera smoothly follows the drone for better visualization.
- The simulation is not a replacement for real-world testing but serves as a tool to approximate and refine PID settings before applying them to the physical drone.

### Summary of Simulation

The simulation provides a virtual environment for testing and calibrating the drone's PID settings. It allows users to adjust parameters like proportional, integral, and derivative constants, as well as the drone's mass and center of mass, in real-time. By visualizing propeller forces, velocity, and orientation, users can refine the drone's stability and behavior before applying changes to the physical hardware. While not a substitute for real-world testing, the simulation serves as a valuable tool for approximating and optimizing flight performance.

## Summary

This project involves the development of a remote-controlled drone using Arduino microcontrollers, basic components, and 3D-printed parts. The software is divided into four main components:

1. **Drone**: The core flight system, implemented on an Arduino MKR Zero, integrates sensors like the MPU 6050 and nRF24L01 radio for communication. It uses custom classes for PID control, motor management, and orientation tracking to ensure stable flight.

2. **Receiver**: Acts as a bridge between the drone and the control panel, using an Arduino UNO and nRF24L01 radio to relay messages. It manages serial communication with the control panel and radio communication with the drone.

3. **Control Panel**: A web-based application that provides an interface for controlling the drone, adjusting PID values, and monitoring telemetry. It supports multiple input methods and includes features for data collection and analysis.

4. **Simulation**: A Unity-based virtual environment for testing and calibrating PID settings. It allows users to visualize and refine the drone's behavior before applying changes to the physical hardware.

The project emphasizes modularity, with reusable classes and data structures, and provides tools for both real-world and simulated testing to optimize drone performance.
