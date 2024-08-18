/*
 * GccApplication5.c
 *
 * Created: 8/16/2024
 * Author : Electromavericks
 */ 

#define F_CPU 16000000UL // 16 MHz clock speed
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


// Define pins for Motor 1
#define MOTOR1_STEP_PIN    PD2
#define MOTOR1_DIR_PIN     PD3

// Define pins for Motor 2
#define MOTOR2_STEP_PIN    PD4
#define MOTOR2_DIR_PIN     PD5

// Define pins for Motor 3
#define MOTOR3_STEP_PIN    PD6
#define MOTOR3_DIR_PIN     PD7

// Define pins for Motor 4
#define MOTOR4_STEP_PIN    PB0
#define MOTOR4_DIR_PIN     PB1

// Define steps per mm
#define STEPS_PER_MM 100

// Define delay between steps in microseconds
#define STEP_DELAY 1000


#define LCD_ADDR 0x27

#define upPin PB3
#define downPin PB4
#define exitPin PB5
#define MenuPin PB2
#define immediatePin PC0

uint8_t prevUpState = 0;
uint8_t prevDownState = 0;
uint8_t prevMenuState = 0;
uint8_t switchPressedUp = 0;
uint8_t switchPressedDown = 0;
uint8_t switchPressedMenu = 0;

enum DisplayState {
	MAIN_MENU,
	CONTINUE,
	INSIDE_MENU,
	LETS_START
} lcd_state = MAIN_MENU;


void TWI_init(void);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t data);
void lcd_send(uint8_t value, uint8_t mode);
void lcd_command(uint8_t command);
void lcd_data(uint8_t data);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(char* str);

uint8_t currentStateChange(uint8_t pin, uint8_t *prevState, uint8_t *switchPressed);



uint8_t currentStateChange(uint8_t pin, uint8_t *prevState, uint8_t *switchPressed) {
	uint8_t currentState = PINB & (1 << pin);
	if (currentState != *prevState) {
		if (currentState) {  // Button is pressed
			if (!(*switchPressed)) {
				*switchPressed = 1;
				*prevState = currentState;  // Update state before returning
				return 1;  // State changed
			}
			} else {
			*switchPressed = 0;  // Reset when the button is released
		}
	}
	*prevState = currentState;  // Save the current state for next comparison
	return 0;  // No state change
}

void TWI_init(void) {
	TWSR = 0x00;
	TWBR = 0x48;
	TWCR = (1 << TWEN);  // Enable TWI
}

void TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Send start condition
	while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag
}

void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);  // Send stop condition
	while (TWCR & (1 << TWSTO));  // Wait for stop to complete
}

void TWI_write(uint8_t data) {
	TWDR = data;  // Load data into TWDR
	TWCR = (1 << TWINT) | (1 << TWEN);  // Start transmission
	while (!(TWCR & (1 << TWINT)));  // Wait for TWINT flag
}

void lcd_send(uint8_t value, uint8_t mode) {
	TWI_start();  // Start I2C communication
	TWI_write(LCD_ADDR << 1);  // Write LCD address

	uint8_t data = (value & 0xF0) | mode | 0x08;  // Send upper nibble
	TWI_write(data);
	TWI_write(data | 0x04);  // Pulse the enable bit
	TWI_write(data & ~0x04);

	data = ((value << 4) & 0xF0) | mode | 0x08;  // Send lower nibble
	TWI_write(data);
	TWI_write(data | 0x04);  // Pulse the enable bit
	TWI_write(data & ~0x04);

	TWI_stop();  // Stop I2C communication
}

void lcd_command(uint8_t command) {
	lcd_send(command, 0);
	_delay_ms(2);
}

void lcd_data(uint8_t data) {
	lcd_send(data, 1);
	_delay_ms(2);
}

void lcd_init(void) {
	_delay_ms(50);  // Wait for LCD to power up
	lcd_command(0x02);  // Initialize LCD in 4-bit mode
	lcd_command(0x28);  // Function set: 4-bit mode, 2 lines, 5x8 dots
	lcd_command(0x0C);  // Display on, cursor off
	lcd_command(0x06);  // Entry mode set: Increment cursor
	lcd_command(0x01);  // Clear display
	_delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address = (row == 0) ? 0x80 : 0xC0;
	address += col;
	lcd_command(address);
}

void lcd_print(char* str) {
	while (*str) {
		lcd_data(*str++);
	}
}


void setup() {
	// Set Motor 1 and Motor 2 pins as output
	DDRD |= (1 << MOTOR1_STEP_PIN) | (1 << MOTOR1_DIR_PIN);
	DDRD |= (1 << MOTOR2_STEP_PIN) | (1 << MOTOR2_DIR_PIN);

	// Set Motor 3 pins as output
	DDRD |= (1 << MOTOR3_STEP_PIN) | (1 << MOTOR3_DIR_PIN);

	// Set Motor 4 pins as output
	DDRB |= (1 << MOTOR4_STEP_PIN) | (1 << MOTOR4_DIR_PIN);

	// Set initial direction for all motors
	PORTD &= ~(1 << MOTOR1_DIR_PIN);
	PORTD &= ~(1 << MOTOR2_DIR_PIN);
	PORTD &= ~(1 << MOTOR3_DIR_PIN);
	PORTB &= ~(1 << MOTOR4_DIR_PIN);
}

void stepMotor(uint8_t stepPin, uint8_t dirPin, uint8_t direction) {
	// Set direction
	if (direction) {
		if (dirPin == MOTOR1_DIR_PIN || dirPin == MOTOR2_DIR_PIN || dirPin == MOTOR3_DIR_PIN)
		PORTD |= (1 << dirPin);
		else if (dirPin == MOTOR4_DIR_PIN)
		PORTB |= (1 << dirPin);
		} else {
		if (dirPin == MOTOR1_DIR_PIN || dirPin == MOTOR2_DIR_PIN || dirPin == MOTOR3_DIR_PIN)
		PORTD &= ~(1 << dirPin);
		else if (dirPin == MOTOR4_DIR_PIN)
		PORTB &= ~(1 << dirPin);
	}
	
	// Trigger step
	if (stepPin == MOTOR1_STEP_PIN || stepPin == MOTOR2_STEP_PIN || stepPin == MOTOR3_STEP_PIN) {
		PORTD |= (1 << stepPin);
		_delay_us(STEP_DELAY);
		PORTD &= ~(1 << stepPin);
		_delay_us(STEP_DELAY);
		} else if (stepPin == MOTOR4_STEP_PIN) {
		PORTB |= (1 << stepPin);
		_delay_us(STEP_DELAY);
		PORTB &= ~(1 << stepPin);
		_delay_us(STEP_DELAY);
	}
}

void moveMotor(uint8_t stepPin, uint8_t dirPin, uint8_t direction, float distance_mm) {
	int steps = distance_mm * STEPS_PER_MM;
	// Debugging message to check direction
	if (dirPin == MOTOR3_DIR_PIN) {
		if (direction) {
			// Check if direction is set to forward
			PORTD |= (1 << MOTOR3_DIR_PIN);
			} else {
			// Check if direction is set to backward
			PORTD &= ~(1 << MOTOR3_DIR_PIN);
		}
	}
	for (int i = 0; i < steps; i++) {
		
		stepMotor(stepPin, dirPin, direction);
	}
}



#define MAX_STEP_DELAY 2000  // Maximum delay in microseconds (slowest speed)
#define MIN_STEP_DELAY 500   // Minimum delay in microseconds (fastest speed)
#define ACCELERATION_RATE 50 // Rate of acceleration (decrease in delay per step)
#define STEPS_PER_MM 100     // Number of steps per mm of movement (motor dependent)

// Function to calculate the maximum of two values
inline uint16_t max(uint16_t a, uint16_t b) {
	return (a > b) ? a : b;
}

// Function to calculate the minimum of two values
inline uint16_t min(uint16_t a, uint16_t b) {
	return (a < b) ? a : b;
}

// Function for a precise delay (in microseconds)
void custom_delay_us(uint16_t delay_us) {
	// Adjust this calculation based on your clock frequency and required precision
	while (delay_us--) {
		_delay_us(1);  // Delay for 1 microsecond
	}
}

// Function to step the motor with a specific delay
void stepMotorWithDelay(uint8_t stepPin, uint8_t dirPin, uint8_t direction, uint16_t delay_us) {
	// Set direction
	if (direction) {
		if (dirPin == MOTOR1_DIR_PIN || dirPin == MOTOR2_DIR_PIN || dirPin == MOTOR3_DIR_PIN)
		PORTD |= (1 << dirPin);
		else if (dirPin == MOTOR4_DIR_PIN)
		PORTB |= (1 << dirPin);
		} else {
		if (dirPin == MOTOR1_DIR_PIN || dirPin == MOTOR2_DIR_PIN || dirPin == MOTOR3_DIR_PIN)
		PORTD &= ~(1 << dirPin);
		else if (dirPin == MOTOR4_DIR_PIN)
		PORTB &= ~(1 << dirPin);
	}

	// Trigger step
	if (stepPin == MOTOR1_STEP_PIN || stepPin == MOTOR2_STEP_PIN || stepPin == MOTOR3_STEP_PIN) {
		PORTD |= (1 << stepPin);
		custom_delay_us(delay_us);
		PORTD &= ~(1 << stepPin);
		} else if (stepPin == MOTOR4_STEP_PIN) {
		PORTB |= (1 << stepPin);
		custom_delay_us(delay_us);
		PORTB &= ~(1 << stepPin);
	}

	// Delay before the next step
	custom_delay_us(delay_us);
}

// Function to move the motor with acceleration and deceleration
void moveMotorWithAcceleration(uint8_t stepPin, uint8_t dirPin, uint8_t direction, float distance_mm) {
	int steps = distance_mm * STEPS_PER_MM;
	int accelSteps = steps / 3;  // 1/3 of steps for acceleration and deceleration
	int decelSteps = min(steps / 3, 200);
	int constSteps = steps - (accelSteps + decelSteps);

	uint16_t delay_us = MAX_STEP_DELAY;  // Start with a max delay (slowest speed)

	// Acceleration phase
	for (int i = 0; i < accelSteps; i++) {
		stepMotorWithDelay(stepPin, dirPin, direction, delay_us);
		delay_us = max(delay_us - ACCELERATION_RATE, MIN_STEP_DELAY);  // Decrease delay to increase speed
	}

	// Constant speed phase
	for (int i = 0; i < constSteps; i++) {
		stepMotorWithDelay(stepPin, dirPin, direction, MIN_STEP_DELAY);  // Move at constant speed
	}
	
	// Deceleration phase
	for (int i = 0; i < decelSteps; i++) {
		delay_us = min(delay_us + ACCELERATION_RATE, MAX_STEP_DELAY);  // Increase delay to decrease speed
		stepMotorWithDelay(stepPin, dirPin, direction, delay_us);
	}
}



int main(void) {
	
	TWI_init();   // Initialize TWI (I2C) and LCD
	lcd_init();
	
	setup();
	
	DDRB &= ~((1 << upPin) | (1 << downPin) | (1 << MenuPin)); // Set pins as input
	DDRC &= ~(1 << immediatePin); // Set immediatePin as input

	prevUpState = PINB & (1 << upPin);
	prevDownState = PINB & (1 << downPin);
	prevMenuState = PINB & (1 << MenuPin);

	lcd_set_cursor(0, 2);
	lcd_print("Pick and Place");  // Start with Menu display
	lcd_set_cursor(1, 2);
	lcd_print("Robot Arm");
	lcd_state = MAIN_MENU;

	while (1) {
		// Handle UP button press
		if (currentStateChange(upPin, &prevUpState, &switchPressedUp)) {
			if (lcd_state == MAIN_MENU) {
				lcd_state = CONTINUE;
				lcd_command(0x01);  // Clear display
				lcd_set_cursor(0, 2);
				lcd_print("Continue");
				} else if (lcd_state == CONTINUE) {
				lcd_state = MAIN_MENU;
				lcd_command(0x01);  // Clear display
				lcd_set_cursor(0, 2);
				lcd_print("Menu");
			}
		}

		// Handle DOWN button press
		else if (currentStateChange(downPin, &prevDownState, &switchPressedDown)) {
			if (lcd_state == MAIN_MENU) {
				lcd_state = CONTINUE;
				lcd_command(0x01);  // Clear display
				lcd_set_cursor(0, 2);
				lcd_print("Continue");
				} else if (lcd_state == CONTINUE) {
				lcd_state = MAIN_MENU;
				lcd_command(0x01);  // Clear display
				lcd_set_cursor(0, 2);
				lcd_print("Menu");
			}
		}

		// Handle MENU button press
		else if (currentStateChange(MenuPin, &prevMenuState, &switchPressedMenu)) {
			lcd_command(0x01);  // Clear display
			if (lcd_state == MAIN_MENU) {
				lcd_set_cursor(0, 2);
				lcd_print("Calibration Loading...");
				lcd_state = INSIDE_MENU;
			}
			else if (lcd_state == CONTINUE) {
				
				lcd_state = LETS_START;
				break;
				
			}
		}
	}

	lcd_set_cursor(0, 0);
	lcd_print("Assembly Process");
	lcd_set_cursor(1, 4);
	lcd_print("Started...");
	
	// Motor 2 forward by 50mm (z-axis) with acceleration and deceleration
	moveMotorWithAcceleration(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 40.0);  // 1 for forward direction
	_delay_ms(1000);

	// Motor 1 backward by 50mm (x-axis) with acceleration and deceleration
	moveMotorWithAcceleration(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 0, 40.0);  // 0 for backward direction
	_delay_ms(1000);
	
	// Motor 2 backward by 50mm (z-axis) with acceleration and deceleration
	moveMotorWithAcceleration(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 40.0);  // 0 for backward direction
	_delay_ms(1000);

	// Motor 1 forward by 50mm (x-axis) with acceleration and deceleration
	moveMotorWithAcceleration(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 1, 40.0);  // 1 for forward direction
	_delay_ms(1000);
	
	while (1)
	{
		lcd_command(0x01);  // Clear display
		lcd_set_cursor(0, 0);
		lcd_print("Assembly Process");
		lcd_set_cursor(1, 4);
		lcd_print("Started...");
		
		//catch ring from motor 4
		moveMotor(MOTOR4_STEP_PIN, MOTOR4_DIR_PIN, 0, 8.0);   // Check direction 1
		_delay_ms(4000);

		
		//motor 1 back 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 1, 10.0);
		_delay_ms(1000);
		
		//motor 2  down by 10 ---y
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 25.0);
		_delay_ms(1000);
		
		//motor 1 forward 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 0, 10.0);
		_delay_ms(1000);
		
		//pivotting
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		moveMotor(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 1, 0.3);//60 degree rotate
		_delay_ms(2000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		moveMotor(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 1, 0.3);//60 degree rotate
		_delay_ms(2000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		moveMotor(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 1, 0.3);//60 degree rotate
		_delay_ms(2000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		moveMotor(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 1, 0.3);//60 degree rotate
		_delay_ms(2000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		moveMotor(MOTOR3_STEP_PIN, MOTOR3_DIR_PIN, 1, 0.3);//60 degree rotate
		_delay_ms(2000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 5.0);//down
		_delay_ms(1000);
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 5.0);//up
		_delay_ms(1000);
		
		
		
		//motor 1 back 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 1, 10.0);
		_delay_ms(1000);
		
		//motor 2  down by 10 ---y
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 0, 15.0);
		_delay_ms(1000);
		
		//motor 1 forward 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 0, 10.0);
		_delay_ms(1000);
		
		//catch out ring from motor 4
		moveMotor(MOTOR4_STEP_PIN, MOTOR4_DIR_PIN, 1, 8.0);   // Check direction 1
		_delay_ms(4000);
		
		//motor 1 back 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 1, 10.0);
		_delay_ms(1000);
		
		//motor 2 up 20
		moveMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, 1, 40.0);
		_delay_ms(1000);
		
		//motor 1 forward 10 ---x
		moveMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, 0, 10.0);
		_delay_ms(1000);

		
	}
	
	
	
	return 0;
}
