
#include <LiquidCrystal.h>
#include <Wire.h>
#include "SpiritLevel.h"


//Declaring some global variables
static boolean			gbSetGyroAngles;

static int				lcd_loop_counter,
						iMainLoopCount,
						angle_pitch_buffer, angle_roll_buffer;

static long				acc_total_vector,
						gyro_x_cal, gyro_y_cal, gyro_z_cal,
						loop_timer;

static float			angle_pitch, angle_roll,
						angle_roll_acc, angle_pitch_acc,
						angle_pitch_output, angle_roll_output;

LiquidCrystal			lcd(7, 8, 9, 10, 11, 12);		// LiquidCrystal(rs, enable, d4, d5, d6, d7)


////////////////////////////////////////////////////////////
//	setup
////////////////////////////////////////////////////////////
void setup() {

	int				gyro_x, gyro_y, gyro_z, temperature;
	long			acc_x, acc_y, acc_z;


	gbSetGyroAngles = FALSE;

	iMainLoopCount = 1;

	Wire.begin();												//Start I2C as master

	Serial.begin(57600);										//Use only for debugging

	pinMode(13, OUTPUT);										//Set output 13 (LED) as output

	InitMPU6050();												// Init the MPU6050 (500dfs / +/-8g)

	digitalWrite(13, HIGH);										// Turn on the LED

	// Init the LCD, write the introduction message for 1.5 seconds, then the cal message
	lcd.begin(16, 2);											// Set the number of columns and rows for the LCD
	lcd.setCursor(0, 0);
	lcd.print("  Digital Level");
	lcd.setCursor(0, 1);
	lcd.print("    Ver. 1.0");
	delay(1500);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Calibrating Gyro");
	lcd.setCursor(0, 1);

	// Average the gyro levels over GYRO_CAL_ITERATIONS cycles to get the cal value
	gyro_x_cal = 0;
	gyro_y_cal = 0;
	gyro_z_cal = 0;
	for (int cal_int = 0; cal_int < GYRO_CAL_ITERATIONS; cal_int++) {
		if (cal_int % GYRO_CAL_DOT_PERIOD == 0)
			lcd.print(".");										// Print a dot on the LCD every 125 readings
		read_mpu_6050_data(gyro_x, gyro_y, gyro_z, temperature, acc_x, acc_y, acc_z);
		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;
		delay(3);												// Delay 3us to simulate the 250Hz program loop
	}
	gyro_x_cal /= GYRO_CAL_ITERATIONS;
	gyro_y_cal /= GYRO_CAL_ITERATIONS;
	gyro_z_cal /= GYRO_CAL_ITERATIONS;

	// Clear the LCD and display the Pitch and Roll tags
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Pitch:");
	lcd.setCursor(0, 1);
	lcd.print("Roll :");

	// Debug print
	Serial.print("X cal: ");Serial.println(gyro_x_cal);
	Serial.print("Y cal: ");Serial.println(gyro_y_cal);
	Serial.print("Z cal: ");Serial.println(gyro_z_cal);
	Serial.println();

	digitalWrite(13, LOW);										// All done, turn the LED off

	lcd_loop_counter = 1;										// Init the LCD loop counter

	loop_timer = micros();										// Reset the loop timer
}


////////////////////////////////////////////////////////////
//	loop
////////////////////////////////////////////////////////////
void loop() {

	int				gyro_x, gyro_y, gyro_z, temperature;
	long			acc_x, acc_y, acc_z;


	// Read the raw acc, temp, and gyro data from the MPU-6050
	read_mpu_6050_data(gyro_x, gyro_y, gyro_z, temperature, acc_x, acc_y, acc_z);

	// Subtract the gyros' offset calibration values from the raw values
	gyro_x -= gyro_x_cal;
	gyro_y -= gyro_y_cal;
	gyro_z -= gyro_z_cal;

	// Debug print
	if (iMainLoopCount < 750)
		iMainLoopCount++;
	else {
		iMainLoopCount = 1;
		Serial.println();
		Serial.print("X: ");Serial.println(gyro_x);
		Serial.print("Y: ");Serial.println(gyro_y);
		Serial.print("Z: ");Serial.println(gyro_z);
		Serial.print("AP: ");Serial.println(acc_x);
		Serial.print("AR: ");Serial.println(acc_y);
		Serial.println();
	}

	// Gyro angle calculations  [0.0000611 = 1 / (250Hz / 65.5)]
	angle_pitch += gyro_x * 0.0000610687;                                // Calculate the traveled pitch angle and add this to the angle_pitch variable
	angle_roll += gyro_y * 0.0000610687;                                 // Calculate the traveled roll angle and add this to the angle_roll variable

	// 0.000001066 = 0.0000611 * (3.142 / 180deg) The Arduino sin function is in radians
	angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               // If the IMU has yawed transfer the roll angle to the pitch angel
	angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               // If the IMU has yawed transfer the pitch angle to the roll angel

	// Accelerometer angle calculations 
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));  // Calculate the total accelerometer vector
																			 // 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
	angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;	// Calculate the pitch angle
	angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;	// Calculate the roll angle
	
	//
	// Place the MPU-6050 spirit level on a level surface and note the values in the following two lines for calibration
	//
	angle_pitch_acc -= 0.0;                                             // Accelerometer calibration value for pitch
	angle_roll_acc -= 0.0;                                              // Accelerometer calibration value for roll

	if (gbSetGyroAngles) {												// If the IMU is already started
		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     // Correct the drift of the gyro roll angle with the accelerometer roll angle
	}
	else {                                                              // At first start
		angle_pitch = angle_pitch_acc;                                  // Set the gyro pitch angle equal to the accelerometer pitch angle
		angle_roll = angle_roll_acc;                                    // Set the gyro roll angle equal to the accelerometer roll angle
		gbSetGyroAngles = TRUE;                                         // Set the IMU started flag
	}

	// To dampen the pitch and roll angles a complementary filter is used
	angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
	angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

	write_LCD();                                                         //Write the roll and pitch values to the LCD display

	while (micros() - loop_timer < LOOP_TIME_MS);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
	loop_timer = micros();                                               //Reset the loop timer
}


////////////////////////////////////////////////////////////
//	read_mpu_6050_data
////////////////////////////////////////////////////////////
void read_mpu_6050_data(int &gy_x, int &gy_y, int &gy_z, int &temp, long &ac_x, long &ac_y, long &ac_z)
{
	// Request 14 bytes from the MPU-6050, starting at the accelerometer x-axis high byte
	Wire.beginTransmission(I2C_ADDR_MPU6050_1);
	Wire.write(MPU6050_REG_ACCEL_XOUT_H);
	Wire.endTransmission();
	Wire.requestFrom(I2C_ADDR_MPU6050_1, 14);

	while (Wire.available() < 14);						// Wait until all the bytes are received

	ac_x = Wire.read() << 8 | Wire.read();       
	ac_y = Wire.read() << 8 | Wire.read();       
	ac_z = Wire.read() << 8 | Wire.read();

	temp = Wire.read() << 8 | Wire.read();

	gy_x = Wire.read() << 8 | Wire.read();      
	gy_y = Wire.read() << 8 | Wire.read();      
	gy_z = Wire.read() << 8 | Wire.read();      
}


////////////////////////////////////////////////////////////
//	write_LCD
////////////////////////////////////////////////////////////
void write_LCD()
{
	switch (lcd_loop_counter)
	{
		case 1:									// Buffer the pitch angle because it will change and set the LCD cursor to initial position on first line
			angle_pitch_buffer = angle_pitch_output * 10;                      
			lcd.setCursor(6, 0);
			break;

		case 2:									// Print + or - as appropriate
			if (angle_pitch_buffer < 0)
				lcd.print(" - ");
			else
				lcd.print(" + ");
			break;

		case 3:									//Print first number
			lcd.print(abs(angle_pitch_buffer) / 1000);    
			break;

		case 4:									//Print second number
			lcd.print((abs(angle_pitch_buffer) / 100) % 10);
			break;

		case 5:									//Print third number
			lcd.print((abs(angle_pitch_buffer) / 10) % 10); 
			break;

		case 6:									//Print decimal point
			lcd.print(".");                             
			break;

		case 7:									//Print decimal number
			lcd.print(abs(angle_pitch_buffer) % 10);      
			break;

		case 8:								// Print degree
			lcd.print((char)223);
			break;

		case 9:									// Buffer the roll angle because it will change and set the LCD cursor to initial position on second line
			angle_roll_buffer = angle_roll_output * 10;
			lcd.setCursor(6, 1);
			break;

		case 10:									// Print + or - as appropriate
			if (angle_roll_buffer < 0)
				lcd.print(" - ");
			else
				lcd.print(" + ");
			break;

		case 11:								//Print first number
			lcd.print(abs(angle_roll_buffer) / 1000);
			break;

		case 12:								//Print second number
			lcd.print((abs(angle_roll_buffer) / 100) % 10);
			break;

		case 13:								//Print third number
			lcd.print((abs(angle_roll_buffer) / 10) % 10);
			break;

		case 14:								//Print decimal point
			lcd.print(".");
			break;

		case 15:								// Print decimal number
			lcd.print(abs(angle_roll_buffer) % 10);
			break;

		case 16:								// Print decimal number
			lcd.print((char)223);
			lcd_loop_counter = 0;
			break;

		default:
			lcd_loop_counter = 0;
			break;
	}

	lcd_loop_counter++;
}


////////////////////////////////////////////////////////////
//	setup_mpu_6050_registers
////////////////////////////////////////////////////////////
void InitMPU6050() {
	
	// Wake it up
	Wire.beginTransmission(I2C_ADDR_MPU6050_1);
	Wire.write(MPU6050_REG_PWR_MGMT_1);
	Wire.write(0x00);
	Wire.endTransmission();
	
	// Configure the accelerometer (+/-8g)
	Wire.beginTransmission(I2C_ADDR_MPU6050_1);
	Wire.write(MPU6050_REG_ACCEL_CONFIG);
	Wire.write(MPU6050_REG_ACCEL_CONFIG_8G_FULL_SCALE);
	Wire.endTransmission();
	
	// Configure the gyro (500dps full scale)
	Wire.beginTransmission(I2C_ADDR_MPU6050_1);
	Wire.write(MPU6050_REG_GYRO_CONFIG);
	Wire.write(0x08);
	Wire.endTransmission();
}