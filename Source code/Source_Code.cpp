#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

// Define F_CPU for _delay_ms()
#define F_CPU 16000000UL

// Pin Definitions
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

// EEPROM Addresses for saving motor positions
uint16_t eepromAddr_stepper1 = 0;
uint16_t eepromAddr_stepper2 = 2;
uint16_t eepromAddr_stepper3 = 4;
uint16_t eepromAddr_stepper4 = 6;

// Variables to store saved positions
int16_t savedPosition1 = 0;
int16_t savedPosition2 = 0;
int16_t savedPosition3 = 0;
int16_t savedPosition4 = 0;

int16_t initial_speed1 = 10;
int16_t initial_speed2 = 10;
int16_t initial_speed3 = 10;
int16_t initial_speed4 = 10;
int16_t initial_x_coordinate = 10;
int16_t initial_z_coordinate = 10;
int16_t default_holes = 6;

int16_t temp_speed1 = 0;
int16_t temp_speed2 = 0;
int16_t temp_speed3 = 10;
int16_t temp_speed4 = 10;
int16_t temp_x_coordinate = 0;
int16_t temp_z_coordinate = 0;
int16_t temp_holes = 0;

// LCD Initialization
void i2c_init() {
    // Initialize I2C (TWI) interface
    TWSR = 0x00;  // Set prescaler to 1
    TWBR = 0x0C;  // Set bit rate register (SCL frequency = F_CPU / (16 + 2 * TWBR * prescaler))
    TWCR = (1 << TWEN);  // Enable TWI
}

void i2c_start() {
    // Send start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));  // Wait for start condition to be transmitted
}

void i2c_stop() {
    // Send stop condition
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));  // Wait for stop condition to be executed
}

void i2c_write(uint8_t data) {
    // Write data to TWI data register
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));  // Wait for data to be transmitted
}

void lcd_command(uint8_t cmd) {
    // Send command to LCD
    i2c_start();
    i2c_write(0x4E);  // LCD I2C address
    i2c_write(0x00);  // Control byte: Co = 0, RS = 0
    i2c_write(cmd);
    i2c_stop();
}

void lcd_data(uint8_t data) {
    // Send data to LCD
    i2c_start();
    i2c_write(0x4E);  // LCD I2C address
    i2c_write(0x40);  // Control byte: Co = 0, RS = 1
    i2c_write(data);
    i2c_stop();
}

void lcd_init() {
    // Initialize LCD
    _delay_ms(50);
    lcd_command(0x38);  // Function set: 8-bit, 2 lines, 5x8 dots
    _delay_ms(5);
    lcd_command(0x0C);  // Display on, cursor off, blink off
    _delay_ms(5);
    lcd_command(0x01);  // Clear display
    _delay_ms(5);
    lcd_command(0x06);  // Entry mode set: increment, no shift
}

void lcd_print(const char* str) {
    // Print string to LCD
    while (*str) {
        lcd_data(*str++);
    }
}

void setupLCD() {
    i2c_init();
    lcd_init();
    lcd_print("Welcome to Pick and Place Robot Arm");
    _delay_ms(5000);
    lcd_command(0x01);  // Clear display
}

// Button Configuration
void setupButtonPins() {
    DDRC &= ~((1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << MENU_BUTTON_PIN) | (1 << CANCEL_BUTTON_PIN) | (1 << IMMEDIATE_BUTTON_PIN));
    PORTC |= (1 << UP_BUTTON_PIN) | (1 << DOWN_BUTTON_PIN) | (1 << MENU_BUTTON_PIN) | (1 << CANCEL_BUTTON_PIN) | (1 << IMMEDIATE_BUTTON_PIN);
}

// Main Menu and Submenus
const char* options_main[] = { "1 - Menu", "2 - Continue" };
int current_main_mode = 0;
const int max_main_modes = 2;

const char* options[] = { "1 - Calibration Mode", "2 - Set Speed", "3 - Number of the holes", "4 - Back" };
const char* sub1_options[] = { "1 - Distance X axis", "2 - Distance Z axis", "3 - Back" };
const char* sub2_options[] = { "1 - Stepper 1", "2 - Stepper 2", "3 - Stepper 3", "4 - Stepper 4", "5 - Back" };

void displayMainMenu() {
    lcd_command(0x01);  // Clear display
    lcd_print(options_main[current_main_mode]);
}

void navigateMenu();
void execute_operation();
bool currentStateChange(uint8_t pin);
void savePositionToEEPROM(uint16_t addr, int16_t value);
int16_t readPositionFromEEPROM(uint16_t addr);
void navigateSub1Menu();
void navigateSub2Menu();
void calibrateXAxis();
void calibrateZAxis();
void setSpeed(int stepper);
void setNumberOfHoles();
void step_motor(volatile uint8_t *step_port, uint8_t step_pin, volatile uint8_t *dir_port, uint8_t dir_pin, uint16_t steps, uint8_t direction);
bool immediate();

void loop() {
    while (1) {
        displayMainMenu();
        _delay_ms(100);

        if (currentStateChange(UP_BUTTON_PIN)) {
            current_main_mode = (current_main_mode + 1) % max_main_modes;
        } else if (currentStateChange(DOWN_BUTTON_PIN)) {
            current_main_mode = (current_main_mode - 1 + max_main_modes) % max_main_modes;
        } else if (currentStateChange(MENU_BUTTON_PIN)) {
            if (current_main_mode == 0) {
                navigateMenu();
            } else {
                execute_operation();
            }
        }
    }
}

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
            if (immediate()) {
                save_positions();
                runloop = 0;
                break;
            }
        }

        
        _delay_ms(2000);

        // Move back to zero
        step_motor(&STEP1_PORT, STEP1_PIN, &DIR1_PORT, DIR1_PIN, 2000, 0);  // Move 2000 steps backward
        step_motor(&STEP2_PORT, STEP2_PIN, &DIR2_PORT, DIR2_PIN, 2000, 0);
        step_motor(&STEP3_PORT, STEP3_PIN, &DIR3_PORT, DIR3_PIN, 200, 0);
        step_motor(&STEP4_PORT, STEP4_PIN, &DIR4_PORT, DIR4_PIN, 20, 0);

        // Wait until movement is complete or immediate button is pressed
        while (runloop) {
            if (immediate()) {
                save_positions();
                runloop = 0;
                break;
            }
        }

        
        _delay_ms(2000);
    }
}

// Helper Functions
bool currentStateChange(uint8_t pin) {
    static uint8_t lastState[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    static uint8_t counter[5] = { 0, 0, 0, 0, 0 };
    uint8_t index = 0;

    switch (pin) {
        case UP_BUTTON_PIN: index = 0; break;
        case DOWN_BUTTON_PIN: index = 1; break;
        case MENU_BUTTON_PIN: index = 2; break;
        case CANCEL_BUTTON_PIN: index = 3; break;
        case IMMEDIATE_BUTTON_PIN: index = 4; break;
    }

    uint8_t currentState = (PINC & (1 << pin)) >> pin;
    if (currentState != lastState[index]) {
        counter[index]++;
        if (counter[index] >= 4) {  // Debounce threshold
            counter[index] = 0;
            lastState[index] = currentState;
            if (currentState == 0) {
                return true;
            }
        }
    } else {
        counter[index] = 0;
    }

    return false;
}

void savePositionToEEPROM(uint16_t addr, int16_t value) {
    eeprom_update_word((uint16_t*)addr, value);
}

int16_t readPositionFromEEPROM(uint16_t addr) {
    return eeprom_read_word((uint16_t*)addr);
}

void navigateMenu() {
    int mode = 0;
    int sub_mode1 = 0;
    int sub_mode2 = 0;

    while (1) {
        lcd_command(0x01);  // Clear display
        lcd_print(options[mode]);
        _delay_ms(100);

        if (currentStateChange(UP_BUTTON_PIN)) {
            mode = (mode + 1) % 4;
        } else if (currentStateChange(DOWN_BUTTON_PIN)) {
            mode = (mode - 1 + 4) % 4;
        } else if (currentStateChange(MENU_BUTTON_PIN)) {
            switch (mode) {
                case 0:
                    while (1) {
                        navigateSub1Menu(&sub_mode1);
                        if (sub_mode1 == 2) break;
                        if (currentStateChange(MENU_BUTTON_PIN)) {
                            switch (sub_mode1) {
                                case 0: calibrateXAxis(); break;
                                case 1: calibrateZAxis(); break;
                            }
                        }
                    }
                    break;
                case 1:
                    while (1) {
                        navigateSub2Menu(&sub_mode2);
                        if (sub_mode2 == 4) break;
                        if (currentStateChange(MENU_BUTTON_PIN)) {
                            switch (sub_mode2) {
                                case 0: setSpeed(1); break;
                                case 1: setSpeed(2); break;
                                case 2: setSpeed(3); break;
                                case 3: setSpeed(4); break;
                            }
                        }
                    }
                    break;
                case 2: setNumberOfHoles(); break;
                case 3: return;
            }
        }
    }
}

void navigateSub1Menu(int* sub_mode1) {
    lcd_command(0x01);  // Clear display
    lcd_print(sub1_options[*sub_mode1]);
    _delay_ms(100);

    if (currentStateChange(UP_BUTTON_PIN)) {
        *sub_mode1 = (*sub_mode1 + 1) % 3;
    } else if (currentStateChange(DOWN_BUTTON_PIN)) {
        *sub_mode1 = (*sub_mode1 - 1 + 3) % 3;
    }
}

void navigateSub2Menu(int* sub_mode2) {
    lcd_command(0x01);  // Clear display
    lcd_print(sub2_options[*sub_mode2]);
    _delay_ms(100);

    if (currentStateChange(UP_BUTTON_PIN)) {
        *sub_mode2 = (*sub_mode2 + 1) % 5;
    } else if (currentStateChange(DOWN_BUTTON_PIN)) {
        *sub_mode2 = (*sub_mode2 - 1 + 5) % 5;
    }
}

void calibrateXAxis() {
    // Calibration for X Axis
    lcd_command(0x01);  // Clear display
    lcd_print("Calibrating X Axis");
    _delay_ms(1000);

    // Implement calibration logic here
}

void calibrateZAxis() {
    // Calibration for Z Axis
    lcd_command(0x01);  // Clear display
    lcd_print("Calibrating Z Axis");
    _delay_ms(1000);

    // Implement calibration logic here
}

void setSpeed(int stepper) {
    // Set Speed for a stepper motor
    lcd_command(0x01);  // Clear display
    lcd_print("Set Speed");
    _delay_ms(1000);

    // Implement speed setting logic here
}

void setNumberOfHoles() {
    // Set Number of Holes
    lcd_command(0x01);  // Clear display
    lcd_print("Set Number of Holes");
    _delay_ms(1000);

    // Implement number of holes setting logic here
}

void step_motor(volatile uint8_t *step_port, uint8_t step_pin, volatile uint8_t *dir_port, uint8_t dir_pin, uint16_t steps, uint8_t direction) {
    // Set direction
    if (direction) {
        *dir_port |= (1 << dir_pin);
    } else {
        *dir_port &= ~(1 << dir_pin);
    }

    // Step the motor
    for (uint16_t i = 0; i < steps; i++) {
        *step_port |= (1 << step_pin);
        _delay_us(500);  // Adjust delay as needed for motor speed
        *step_port &= ~(1 << step_pin);
        _delay_us(500);  // Adjust delay as needed for motor speed
    }
}

bool immediate() {
    return !currentStateChange(IMMEDIATE_BUTTON_PIN);
}

// Save current positions to EEPROM
void save_positions() {
    savePositionToEEPROM(eepromAddr_stepper1, savedPosition1);
    savePositionToEEPROM(eepromAddr_stepper2, savedPosition2);
    savePositionToEEPROM(eepromAddr_stepper3, savedPosition3);
    savePositionToEEPROM(eepromAddr_stepper4, savedPosition4);
}

int main() {
    // Set direction and step pins as output
    DDRD |= (1 << DIR1_PIN) | (1 << STEP1_PIN) | (1 << DIR2_PIN) | (1 << STEP2_PIN) | (1 << DIR3_PIN) | (1 << STEP3_PIN);
    DDRB |= (1 << DIR4_PIN) | (1 << STEP4_PIN);

    // Initialize LCD and buttons
    setupLCD();
    setupButtonPins();

    // Initialize motor positions from EEPROM
    savedPosition1 = readPositionFromEEPROM(eepromAddr_stepper1);
    savedPosition2 = readPositionFromEEPROM(eepromAddr_stepper2);
    savedPosition3 = readPositionFromEEPROM(eepromAddr_stepper3);
    savedPosition4 = readPositionFromEEPROM(eepromAddr_stepper4);

    sei();  // Enable global interrupts

    // Main loop
    loop();

    return 0;
}
