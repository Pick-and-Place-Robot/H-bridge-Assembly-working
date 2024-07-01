# Pick-and-Place Robot Arm

## Introduction
This project involves the design and implementation of a pick-and-place robot arm. The robot arm is controlled using stepper motors and an LCD interface, with various settings and calibration modes accessible through push buttons.

## Hardware Setup
- **Microcontroller:** ATmega328P (Arduino Uno)
- **Stepper Motors:** 4 stepper motors with drivers
- **LCD:** I2C LCD (16x2)
- **Push Buttons:** 5 buttons for navigation and immediate stop

## Software Setup

### Libraries Used
- `Wire.h`: For I2C communication with the LCD

### Installation
1. Install the required libraries in the Arduino IDE:
    - `Wire.h`
2. Upload the provided code to the ATmega328P using an ISP programmer.

## Code Explanation

### Pin Definitions

```cpp
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define F_CPU 16000000UL

#define DIR1_PORT PORTD
#define DIR1_PIN PD3
#define STEP1_PORT PORTD
#define STEP1_PIN PD2

#define DIR2_PORT PORTD
#define DIR2_PIN PD5
#define STEP2_PORT PORTD
#define STEP2_PIN PD4

#define DIR3_PORT PORTD
#define DIR3_PIN PD7
#define STEP3_PORT PORTD
#define STEP3_PIN PD6

#define DIR4_PORT PORTB
#define DIR4_PIN PB1
#define STEP4_PORT PORTB
#define STEP4_PIN PB0

#define UP_BUTTON_PIN PC0
#define DOWN_BUTTON_PIN PC1
#define MENU_BUTTON_PIN PC2
#define CANCEL_BUTTON_PIN PC3
#define IMMEDIATE_BUTTON_PIN PC4

```

These definitions set the pin numbers for the stepper motors and push buttons.

### EEPROM Setup
```cpp

uint16_t eepromAddr_stepper1 = 0;
uint16_t eepromAddr_stepper2 = 2;
uint16_t eepromAddr_stepper3 = 4;
uint16_t eepromAddr_stepper4 = 6;

int16_t savedPosition1 = 0;
int16_t savedPosition2 = 0;
int16_t savedPosition3 = 0;
int16_t savedPosition4 = 0;
```
EEPROM addresses are defined for saving and retrieving motor positions when the immediate button is pressed.

### Button Configuration
``` cpp

void setupButtonPins() {
    // Set button pins as inputs with pull-up resistors
    DDRB &= ~((1 << UP_PIN) | (1 << DOWN_PIN) | (1 << MENU_PIN) | (1 << CANCEL_PIN));
    PORTB |= (1 << UP_PIN) | (1 << DOWN_PIN) | (1 << MENU_PIN) | (1 << CANCEL_PIN);
    DDRC &= ~(1 << IMMEDIATE_PIN);
    PORTC |= (1 << IMMEDIATE_PIN);
}
```
This function configures the button pins as inputs with pull-up resistors.

### LCD Initialization
```cpp

#include <Wire.h>
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setupLCD() {
    lcd.init();
    lcd.backlight();
    lcd.print("Welcome to Pick and Place Robot Arm");
    _delay_ms(5000);
    lcd.clear();
}
```
The LCD is initialized and a welcome message is displayed.

### Display Menu Navigation
```cpp

const char* options_main[] = { "1 - Menu", "2 - Continue" };
int current_main_mode = 0;
const int max_main_modes = 2;

void displayMainMenu() {
    lcd.clear();
    lcd.print(options_main[current_main_mode]);
}
```
Main menu options are defined with navigation logic implemented in the setup() function.

### Submenus
```cpp

const char* options[] = { "1 - Calibration Mode", "2 - Set Speed", "3 - Number of the holes", "4 - Back" };
const char* sub1_options[] = { "1 - Distance X axis", "2 - Distance Z axis", "3 - Back" };
const char* sub2_options[] = { "1 - Stepper 1", "2 - Stepper 2", "3 - Stepper 3", "4 - Stepper 4", "5 - Back" };
```
Submenu options for calibration and speed settings are defined with corresponding navigation logic.

### Loop Function

```cpp
void loop() {
    while (1) {
        displayMainMenu();
        _delay_ms(100);

        if (currentStateChange(UP_PIN)) {
            current_main_mode = (current_main_mode + 1) % max_main_modes;
        } else if (currentStateChange(DOWN_PIN)) {
            current_main_mode = (current_main_mode - 1 + max_main_modes) % max_main_modes;
        } else if (currentStateChange(MENU_PIN)) {
            if (current_main_mode == 0) {
                navigateMenu();
            } else {
                executeOperation();
            }
        }
    }
}
```
The loop() function handles the main operations of the robot arm, moving the steppers to predefined positions.

### Main Operation Code
```cpp
void execute_operation() {
  int runloop = 1;

  while (runloop) {
    // Move to positions
    step_motor(&STEP1_PORT, STEP1_PIN, &DIR1_PORT, DIR1_PIN, 2000, 1);  // Move 2000 steps forward
    step_motor(&STEP2_PORT, STEP2_PIN, &DIR2_PORT, DIR2_PIN, 2000, 1);
    step_motor(&STEP3_PORT, STEP3_PIN, &DIR3_PORT, DIR3_PIN, 200, 1);
    step_motor(&STEP4_PORT, STEP4_PIN, &DIR4_PORT, DIR4_PIN, 20, 1);

    // Wait until movement is complete or immediate button is pressed
    while (runloop) {
      if (immidiate()) {
        runloop = 0;
        break;
      }
    }

    save_positions();
    _delay_ms(2000);

    // Move back to zero
    step_motor(&STEP1_PORT, STEP1_PIN, &DIR1_PORT, DIR1_PIN, 2000, 0);  // Move 2000 steps backward
    step_motor(&STEP2_PORT, STEP2_PIN, &DIR2_PORT, DIR2_PIN, 2000, 0);
    step_motor(&STEP3_PORT, STEP3_PIN, &DIR3_PORT, DIR3_PIN, 200, 0);
    step_motor(&STEP4_PORT, STEP4_PIN, &DIR4_PORT, DIR4_PIN, 20, 0);

    // Wait until movement is complete or immediate button is pressed
    while (runloop) {
      if (immidiate()) {
        runloop = 0;
        break;
      }
    }

    save_positions();
    _delay_ms(2000);
  }
}
```

### Helper Functions
```cpp

bool currentStateChange(uint8_t pin) {
    static uint8_t lastState = 0xFF;
    uint8_t currentState = PINB & (1 << pin);
    if (currentState != lastState) {
        lastState = currentState;
        if (currentState == 0) {
            return true;
        }
    }
    return false;
}

void savePositionToEEPROM(uint16_t addr, int16_t value) {
    eeprom_write_word((uint16_t*)addr, value);
}

int16_t readPositionFromEEPROM(uint16_t addr) {
    return eeprom_read_word((uint16_t*)addr);
}

void navigateMenu() {
    int current_sub_mode = 0;
    const int max_sub_modes = 4;
    while (1) {
        lcd.clear();
        lcd.print(options[current_sub_mode]);
        _delay_ms(100);

        if (currentStateChange(UP_PIN)) {
            current_sub_mode = (current_sub_mode + 1) % max_sub_modes;
        } else if (currentStateChange(DOWN_PIN)) {
            current_sub_mode = (current_sub_mode - 1 + max_sub_modes) % max_sub_modes;
        } else if (currentStateChange(MENU_PIN)) {
            if (current_sub_mode == 0) {
                navigateSub1Menu();
            } else if (current_sub_mode == 1) {
                navigateSub2Menu();
            } else if (current_sub_mode == 2) {
                setNumberOfHoles();
            } else {
                break;
            }
        }
    }
}

void navigateSub1Menu() {
    int current_sub1_mode = 0;
    const int max_sub1_modes = 3;
    while (1) {
        lcd.clear();
        lcd.print(sub1_options[current_sub1_mode]);
        _delay_ms(100);

        if (currentStateChange(UP_PIN)) {
            current_sub1_mode = (current_sub1_mode + 1) % max_sub1_modes;
        } else if (currentStateChange(DOWN_PIN)) {
            current_sub1_mode = (current_sub1_mode - 1 + max_sub1_modes) % max_sub1_modes;
        } else if (currentStateChange(MENU_PIN)) {
            if (current_sub1_mode == 0) {
                calibrateXAxis();
            } else if (current_sub1_mode == 1) {
                calibrateZAxis();
            } else {
                break;
            }
        }
    }
}

void navigateSub2Menu() {
    int current_sub2_mode = 0;
    const int max_sub2_modes = 5;
    while (1) {
        lcd.clear();
        lcd.print(sub2_options[current_sub2_mode]);
        _delay_ms(100);

        if (currentStateChange(UP_PIN)) {
            current_sub2_mode = (current_sub2_mode + 1) % max_sub2_modes;
        } else if (currentStateChange(DOWN_PIN)) {
            current_sub2_mode = (current_sub2_mode - 1 + max_sub2_modes) % max_sub2_modes;
        } else if (currentStateChange(MENU_PIN)) {
            if (current_sub2_mode == 0) {
                setSpeed(1);
            } else if (current_sub2_mode == 1) {
                setSpeed(2);
            } else if (current_sub2_mode == 2) {
                setSpeed(3);
            } else if (current_sub2_mode == 3) {
                setSpeed(4);
            } else {
                break;
            }
        }
    }
}

void calibrateXAxis() {
    lcd.clear();
    lcd.print("Calibrating X Axis");
    _delay_ms(2000);
    savePositionToEEPROM(eepromAddr_stepper1, savedPosition1);
}

void calibrateZAxis() {
    lcd.clear();
    lcd.print("Calibrating Z Axis");
    _delay_ms(2000);
    savePositionToEEPROM(eepromAddr_stepper2, savedPosition2);
}

void setSpeed(int stepper) {
    lcd.clear();
    lcd.print("Setting Speed for Stepper ");
    lcd.print(stepper);
    _delay_ms(2000);
}

void setNumberOfHoles() {
    lcd.clear();
    lcd.print("Setting Number of Holes");
    _delay_ms(2000);
}
```

### Conclusion
This project demonstrates the basics of a pick-and-place robot arm using an ATmega328P microcontroller. The robot arm can be calibrated, and its speed can be set through an LCD interface and push buttons.

### License
This project is licensed under the MIT License.

