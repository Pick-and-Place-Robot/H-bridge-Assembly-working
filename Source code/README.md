# Pick and Place Robot Arm

This project is an AVR microcontroller-based pick and place robot arm. The robot arm is controlled by stepper motors, and an LCD is used for displaying information. The system allows the user to navigate through different menus to calibrate the robot, set the speed of the stepper motors, and set the number of holes to be drilled. The project also saves and retrieves the positions of the stepper motors from EEPROM.

## Features
- Control four stepper motors for horizontal, vertical, rotational, and gripper movement.
- LCD display for user interaction.
- Save and load stepper motor positions from EEPROM.
- Configure motor speed and number of holes.
- Immediate stop button functionality.

## Hardware Setup
- **Microcontroller**: AVR ATmega series
- **LCD**: I2C LCD Display
- **Stepper Motors**: Four stepper motors connected to respective pins
- **Buttons**: Up, Down, Menu, Cancel, Immediate Stop buttons
- 

## Pin Configuration
- **Stepper Motors**:
  - Motor 1 (Horizontal): Direction - PD3, Step - PD2
  - Motor 2 (Vertical): Direction - PD5, Step - PD4
  - Motor 3 (Rotational): Direction - PD7, Step - PD6
  - Motor 4 (Gripper): Direction - PB1, Step - PB0
- **Buttons**:
  - Up Button: PC0
  - Down Button: PC1
  - Menu Button: PC2
  - Cancel Button: PC3
  - Immediate Stop Button: PC4

## Parameters
- Maximum Speed: 200 steps per second
- Acceleration Rate: 10 steps per second²
- Deceleration Rate: 10 steps per second²

## EEPROM Addresses
- Motor 1 Position: Address 0
- Motor 2 Position: Address 2
- Motor 3 Position: Address 4
- Motor 4 Position: Address 6

## Installation
1. Clone the repository or download the source code.
2. Open the project in your favorite AVR development environment (e.g., Atmel Studio).
3. Connect the hardware as per the pin configuration.
4. Compile and upload the code to the AVR microcontroller.

## Usage
### Main Menu
- **1 - Menu**: Navigate to the main menu.
- **2 - Continue**: Execute the pick and place operation.


### Main Menu Options
- **1 - Calibration Mode**: Calibrate the robot arm.
- **2 - Set Speed**: Set the speed for each stepper motor.
- **3 - Number of the holes**: Set the number of holes to be drilled.
- **4 - Back**: Return to the main menu.

### Calibration Mode
- **1 - Distance X axis**: Calibrate the horizontal distance.
- **2 - Distance Z axis**: Calibrate the verticle distance.

### Set Speed
- **1 - Set Speed for Motor 1**: Set the speed for the horizontal movement.
- **2- Set Speed for Motor 2**: Set the speed for the vertical movement.
- **3 - Set Speed for Motor 3**: Set the speed for the rotational movement.
- **4 - Set Speed for Motor 4**: Set the speed for the gripper.
- **5 - Back***: Return to the main menu.

### Number of Holes
1 - Increase Number of Holes: Increase the number of holes to be drilled.
2 - Decrease Number of Holes: Decrease the number of holes to be drilled.
3 - Save and Back: Save the settings and return to the main menu.


### Operation
Power on the robot arm: Ensure all connections are secure and power on the system.
Navigate the Menu: Use the buttons to navigate through the menu and configure the settings.
Calibrate: Perform calibration if needed.
Set Speed and Number of Holes: Adjust the speed of the motors and set the number of holes.
Execute Operation: Once settings are configured, select "Continue" from the main menu to start the pick and place operation.

## Code Explanation

### Initialization
In the `setup()` function, initialize the LCD, buttons, and stepper motors:

```c
void setup() {
    // Initialize LCD
    lcd.begin(16, 2);
    lcd.print("Initializing...");

    // Initialize buttons
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);

    // Initialize stepper motors
    stepper1.setSpeed(initial_speed);
    stepper2.setSpeed(initial_speed);
    stepper3.setSpeed(initial_speed);
    stepper4.setSpeed(initial_speed);

    lcd.clear();
    lcd.print("Ready");
}
```
### Menu Navigation
Implement the menu navigation using button inputs to select various options:

```c

void loop() {
    if (digitalRead(BUTTON1_PIN) == LOW) {
        // Navigate to Calibration Mode
        calibrationMode();
    } else if (digitalRead(BUTTON2_PIN) == LOW) {
        // Navigate to Set Speed Mode
        setSpeedMode();
    } else if (digitalRead(BUTTON3_PIN) == LOW) {
        // Navigate to Number of Holes
        numberOfHolesMode();
    } else if (digitalRead(BUTTON4_PIN) == LOW) {
        // Execute Operation
        executeOperation();
    }
}
```
### Calibration Mode
Allow the user to manually control each motor to set the initial positions:

```c

void calibrationMode() {
    lcd.clear();
    lcd.print("Calibration Mode");
    // Code to manually control motors
    // ...
}
```
### Set Speed
Set the speed for each stepper motor:

```c

void setSpeedMode() {
    lcd.clear();
    lcd.print("Set Speed");
    // Code to adjust motor speeds
    // ...
    stepper1.setSpeed(new_speed1);
    stepper2.setSpeed(new_speed2);
    stepper3.setSpeed(new_speed3);
    stepper4.setSpeed(new_speed4);
}
Number of Holes
Set the number of holes for the pick and place operation:

c
Copy code
void numberOfHolesMode() {
    lcd.clear();
    lcd.print("Number of Holes");
    // Code to set the number of holes
    // ...
    numberOfHoles = new_number;
}
```
### Execute Operation
Highlight the main execution operation including acceleration, constant speed, and deceleration for the stepper motors:

```c

void executeOperation() {
    lcd.clear();
    lcd.print("Executing...");

    for (int i = 0; i < numberOfHoles; i++) {
        // Move to initial position
        moveToPosition(initial_position);

        // Pick operation
        operateGripper(OPEN);
        delay(1000); // Wait for gripper to open

        // Move to pick position
        moveToPosition(pick_position);
        operateGripper(CLOSE);
        delay(1000); // Wait for gripper to close

        // Move to place position
        moveToPosition(place_position);
        operateGripper(OPEN);
        delay(1000); // Wait for gripper to open

        // Move back to initial position
        moveToPosition(initial_position);
    }

    lcd.clear();
    lcd.print("Done");
}
```
### Acceleration, Constant Speed, and Deceleration
To control the stepper motors with acceleration, constant speed, and deceleration:

```c

void moveToPosition(Position pos) {
    // Example function to move stepper1
    int steps = calculateSteps(current_position, pos);
    
    // Accelerate
    for (int i = 0; i < acceleration_steps; i++) {
        stepper1.setSpeed(min_speed + i * (max_speed - min_speed) / acceleration_steps);
        stepper1.step(1);
    }

    // Constant speed
    for (int i = 0; i < steps - (acceleration_steps + deceleration_steps); i++) {
        stepper1.setSpeed(max_speed);
        stepper1.step(1);
    }

    // Decelerate
    for (int i = 0; i < deceleration_steps; i++) {
        stepper1.setSpeed(max_speed - i * (max_speed - min_speed) / deceleration_steps);
        stepper1.step(1);
    }

    current_position = pos;
}

```



