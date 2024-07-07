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
## Functions

### execute_operation

Coordinates the sequence of movements for the robot arm:

1. Moves the arm horizontally by 40 cm.
2. Moves the arm vertically by 30 cm.
3. Repeats movements to pick and place items, rotating the arm between actions.
4. Continues until an immediate action is triggered.
```c

void execute_operation() {
    
    stepper_to_horizontal(40);  // Move stepper 1 horizontally by 40 cm
    stepper_to_vertical(30);    // Move stepper 2 vertically by 30cm
    
    while (runloop) {
        // Execute the sequence of movements
        stepper_to_fingers(5);
        _delay_ms(1000); 
        stepper_to_horizontal(-5);
        _delay_ms(1000); 
        stepper_to_vertical(-10);
        _delay_ms(1000); 

        stepper_to_horizontal(5);
        _delay_ms(1000); 
        for (int i=0,I<holes,i++){
            stepper_to_rotation(30); 
            _delay_ms(2000); 
        }
        stepper_to_horizontal(-5);
        _delay_ms(1000); 

        stepper_to_vertical(20);
        _delay_ms(1000); 
        stepper_to_horizontal(5);
        _delay_ms(1000); 
        stepper_to_fingers(-5);
        _delay_ms(1000); 

        stepper_to_vertical(-10);

        _delay_ms(5000);  // Wait for 5 seconds 
    }
}

void stepper_to_horizontal(int distance_cm) {
    // Calculate steps based on distance (convert cm to steps)
    // 360 degrees of rotation corresponds to 2 cm of linear distance
    float distance_per_rotation_cm = 2.0; 
    float steps_per_cm = 200.0;  // full step mode
    
    int steps = distance_cm * steps_per_cm / distance_per_rotation_cm;

    // Move the motor
    step_motor(&STEP1_PORT, STEP1_PIN, &DIR1_PORT, DIR1_PIN, steps, 1);
}


// Function to move stepper motor 2 vertically
void stepper_to_vertical(int distance) {
    /// Calculate steps based on distance (convert cm to steps)
    // 360 degrees of rotation corresponds to 2 cm of linear distance
    float distance_per_rotation_cm = 2.0; 
    float steps_per_cm = 200.0;  // full step mode
    
    int steps = distance_cm * steps_per_cm / distance_per_rotation_cm;
    step_motor(&STEP2_PORT, STEP2_PIN, &DIR2_PORT, DIR2_PIN, steps, 1);
}

// Function to rotate stepper motor 3 by a specified angle
void stepper_to_rotation(int angle) {
    // Calculate steps based on angle (convert degrees to steps)
    int steps = angle*200/360;  

    // Move the motor
    step_motor(&STEP3_PORT, STEP3_PIN, &DIR3_PORT, DIR3_PIN, steps, 1);
}

// Function to move stepper motor 4 for fingers or gripper movement
void stepper_to_fingers(int distance) {
     /// Calculate steps based on distance (convert cm to steps)
    // 360 degrees of rotation corresponds to 2 cm of linear distance
    float distance_per_rotation_cm = 1.0; 
    float steps_per_cm = 200.0;  // full step mode
    
    int steps = distance_cm * steps_per_cm / distance_per_rotation_cm;
    step_motor(&STEP4_PORT, STEP4_PIN, &DIR4_PORT, DIR4_PIN, steps, 1);
}

```

### Acceleration and Deceleration:

Handles stepper motor movement with acceleration and deceleration:

```c
void step_motor(volatile uint8_t *step_port, uint8_t step_pin, volatile uint8_t *dir_port, uint8_t dir_pin, uint16_t steps, uint8_t direction) {
    if (immediate()) {
        save_positions();  // Save current positions
        runloop = 0;       // Exit the loop
        return;
    }

    // Set direction
    if (direction) {
        *dir_port |= (1 << dir_pin);
    } else {
        *dir_port &= ~(1 << dir_pin);
    }

    // Calculate the number of steps for each phase
    uint16_t accel_steps = MAX_SPEED / ACCEL_RATE;
    uint16_t decel_steps = MAX_SPEED / DECEL_RATE;
    uint16_t constant_steps = steps - (accel_steps + decel_steps);

    // Acceleration phase
    for (uint16_t i = 0; i < accel_steps; i++) {
        *step_port |= (1 << step_pin);
        _delay_us(1000000 / (ACCEL_RATE * i + 1));
        *step_port &= ~(1 << step_pin);
        _delay_us(1000000 / (ACCEL_RATE * i + 1));
    }

    // Constant speed phase
    for (uint16_t i = 0; i < constant_steps; i++) {
        *step_port |= (1 << step_pin);
        _delay_us(1000000 / MAX_SPEED);
        *step_port &= ~(1 << step_pin);
        _delay_us(1000000 / MAX_SPEED);
    }

    // Deceleration phase
    for (uint16_t i = decel_steps; i > 0; i--) {
        *step_port |= (1 << step_pin);
        _delay_us(1000000 / (DECEL_RATE * i + 1));
        *step_port &= ~(1 << step_pin);
        _delay_us(1000000 / (DECEL_RATE * i + 1));
    }
}
```
