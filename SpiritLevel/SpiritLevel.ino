
#include <LiquidCrystal.h>
#include <Wire.h>
#include "SpiritLevel.h"


//Declaring some global variables
static boolean			set_gyro_angles;

static int				gyro_x, gyro_y, gyro_z,
						temperature,
						lcd_loop_counter,
						angle_pitch_buffer, angle_roll_buffer;

static long				acc_x, acc_y, acc_z, acc_total_vector,
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

	Wire.begin();												//Start I2C as master

	//Serial.begin(57600);										//Use only for debugging

	pinMode(13, OUTPUT);										//Set output 13 (LED) as output

	InitMPU6050();												// Init the MPU6050 (500dfs / +/-8g)

	digitalWrite(13, HIGH);										// Turn on the LED

	// Init the LCD, write the introduction message for 1.5 seconds, then the cal message
	lcd.begin(16, 2);											// Set the number of columns and rows for the LCD
	lcd.setCursor(0, 0);
	lcd.print("Spirit Level");
	lcd.setCursor(0, 1);
	lcd.print("     V1.0");
	delay(1500);
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Calibrating gyro");
	lcd.setCursor(0, 1);

	// Average the gyro levels over GYRO_CAL_ITERATIONS cycles to get the cal value
	gyro_x_cal = 0;
	gyro_y_cal = 0;
	gyro_z_cal = 0;
	for (int cal_int = 0; cal_int < GYRO_CAL_ITERATIONS; cal_int++) {
		if (cal_int % GYRO_CAL_DOT_PERIOD == 0)
			lcd.print(".");										// Print a dot on the LCD every 125 readings
		read_mpu_6050_data();
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

	digitalWrite(13, LOW);										// All done, turn the LED off

	lcd_loop_counter = 1;										// Init the LCD loop counter

	loop_timer = micros();										// Reset the loop timer
}


////////////////////////////////////////////////////////////
//	loop
////////////////////////////////////////////////////////////
void loop() {

	read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

	gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
	gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
	gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

	//Gyro angle calculations  [0.0000611 = 1 / (250Hz / 65.5)]
	angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
	angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable

	// 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
	angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
	angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

	// Accelerometer angle calculations
	acc_total_vector = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));  //Calculate the total accelerometer vector
																			 //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
	angle_pitch_acc = asin((float)acc_y / acc_total_vector)* 57.296;	//Calculate the pitch angle
	angle_roll_acc = asin((float)acc_x / acc_total_vector)* -57.296;	//Calculate the roll angle

	// Place the MPU-6050 spirit level on a level surface and note the values in the following two lines for calibration
	angle_pitch_acc -= 0.0;                                             //Accelerometer calibration value for pitch
	angle_roll_acc -= 0.0;                                              //Accelerometer calibration value for roll

	if (set_gyro_angles) {												//If the IMU is already started
		angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
		angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     //Correct the drift of the gyro roll angle with the accelerometer roll angle
	}
	else {                                                              //At first start
		angle_pitch = angle_pitch_acc;                                  //Set the gyro pitch angle equal to the accelerometer pitch angle
		angle_roll = angle_roll_acc;                                    //Set the gyro roll angle equal to the accelerometer roll angle
		set_gyro_angles = true;                                         //Set the IMU started flag
	}

	//To dampen the pitch and roll angles a complementary filter is used
	angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
	angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

	write_LCD();                                                         //Write the roll and pitch values to the LCD display

	while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
	loop_timer = micros();                                               //Reset the loop timer
}


////////////////////////////////////////////////////////////
//	read_mpu_6050_data
////////////////////////////////////////////////////////////
void read_mpu_6050_data()
{

	Wire.beginTransmission(0x68);
	Wire.write(0x3B);
	Wire.endTransmission();
	Wire.requestFrom(0x68, 14);							// Request 14 bytes from the MPU-6050

	while (Wire.available() < 14);						// Wait until all the bytes are received

	acc_x = Wire.read() << 8 | Wire.read();       
	acc_y = Wire.read() << 8 | Wire.read();       
	acc_z = Wire.read() << 8 | Wire.read();

	temperature = Wire.read() << 8 | Wire.read();

	gyro_x = Wire.read() << 8 | Wire.read();      
	gyro_y = Wire.read() << 8 | Wire.read();      
	gyro_z = Wire.read() << 8 | Wire.read();      
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

		case 8:									// Buffer the roll angle because it will change and set the LCD cursor to initial position on second line
			angle_roll_buffer = angle_roll_output * 10;
			lcd.setCursor(6, 1);
			break;

		case 9:									// Print + or - as appropriate
			if (angle_roll_buffer < 0)
				lcd.print(" - ");
			else
				lcd.print(" + ");
			break;

		case 10:								//Print first number
			lcd.print(abs(angle_roll_buffer) / 1000);
			break;

		case 11:								//Print second number
			lcd.print((abs(angle_roll_buffer) / 100) % 10);
			break;

		case 12:								//Print third number
			lcd.print((abs(angle_roll_buffer) / 10) % 10);
			break;

		case 13:								//Print decimal point
			lcd.print(".");
			break;

		case 14:								// Print decimal number
			lcd.print(abs(angle_roll_buffer) % 10);
			lcd_loop_counter = 1;
			break;

		default:
			lcd_loop_counter = 1;
			break;
		}
}


////////////////////////////////////////////////////////////
//	setup_mpu_6050_registers
////////////////////////////////////////////////////////////
void InitMPU6050() {
	
	// Wake it up
	Wire.beginTransmission(0x68);
	Wire.write(0x6B);
	Wire.write(0x00);
	Wire.endTransmission();
	
	// Configure the accelerometer (+/-8g)
	Wire.beginTransmission(0x68);
	Wire.write(0x1C);
	Wire.write(0x10);
	Wire.endTransmission();
	
	// Configure the gyro (500dps full scale)
	Wire.beginTransmission(0x68);
	Wire.write(0x1B);
	Wire.write(0x08);
	Wire.endTransmission();
}