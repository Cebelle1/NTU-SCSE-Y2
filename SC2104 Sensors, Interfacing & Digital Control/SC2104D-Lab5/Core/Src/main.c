/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  for ver D STM32 board - Lab 5
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "MPU6050.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;  // for the two servor motors

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t IMU_ADDR = 0x68<<1;  //MPU-6050 address
IMU_Data imu;	// see MPU6050.h declaration

float pitch_acc = 0, roll_acc = 0;   // pitch and roll angles measured by Acc
float pitch_acc_LPF=0, roll_acc_LPF =0, pitch_prev=0, roll_prev=0; // For Lowpass filtering of accelerometer
float pitch_gyro=0,  roll_gyro=0, yaw_gyro = 0;  // angles from Gyro data
float pitch_CF = 0,  roll_CF = 0; // angles compute using Complementary filter

// the following are for Kalman Filter
float Var_gyro = 4*4; // uncertainty of gyroscope measurement - not changing over time
float Var_acc = 3*3; // uncertainty of accelerometer measurement - not changing over time
float roll_var_g = 2*2; // initial uncertainty - will be updated
float pitch_var_g = 2*2; // initial uncertainty - will be updated
float KG_roll, KG_pitch; // Kalman Gain, will be calculated
float roll_KF = 0; // estimate roll by KF
float pitch_KF = 0;


char sbuf[10][20];  // buffers to store up to 10 data for serial port transmission, each of max len of 20 char

char temp[10][10]; // temporary buffer to store IMU data as string and display on OLED
                    // first 3 store Acc aX, aY, aZ
                    // next 3 store Gyro gX, gY, gZ
					// next 3 store Magntometer (NA for MPU-6050)
                    // next 1 store Temp, or use as scratch variable

uint32_t millisOld, millisNow;  // to store timer value for gyro angles computation
float dt; // time elapse in between sample

int32_t pitch_servo_target = 65;  // servo motor default PWM values
int32_t roll_servo_target = 150;

//***************************************************************************************

void OLED_disp_msg1()
{ // show starting messsage
  char *buf; // buffer to store value to be display on OLED/SerialPort
  OLED_Init();
  OLED_ShowString(10, 5, "SC2104/CE3002"); // show message on OLED display at line 5)
  buf = "Lab 5"; // another way to show message through buffer
  OLED_ShowString(10,30, buf); //another message at line 30
  buf = "IMU + PID"; // anther way to show message through buffer
  OLED_ShowString(10,50, buf); //another message at line 50
  OLED_Refresh_Gram();
  HAL_Delay(2000); // pause for 2 second to show message

  // send a message to serial port - for testing
  buf = "SC2104\n\r";
  HAL_UART_Transmit(&huart3, buf, 8, HAL_MAX_DELAY); // Send through Serial Port @115200
  HAL_UART_Transmit(&huart2, buf, 8, HAL_MAX_DELAY); // Send through BT Serial Port @9600
  }

void OLED_disp_msg2()
{ // showing messages while adjusting the platform
  OLED_Clear();
  OLED_ShowString(30, 20, "Testing"); // show message on OLED display at line 20)
  OLED_ShowString(10, 30, " Server motors"); // show message on OLED display at line 40)
  OLED_Refresh_Gram();
  Server_Motor12(); // testing servermotor 1 and 2

  // show starting message
  OLED_Clear();
  OLED_ShowString(0, 0,  "Adjust Platform"); // show message on OLED display at line 30)
  OLED_ShowString(0, 10, "    for"); // show message on OLED display at line 40)
  OLED_ShowString(0, 20, "level setting"); // show message on OLED display at line 40)
  //OLED_ShowString(0, 30, "  setting"); // show message on OLED display at line 40)
  OLED_Refresh_Gram();
  HAL_Delay(1000); // pause for 2 second to show message
  millisOld = HAL_GetTick(); // get first time value before starting - for Gyro dt compuattaion
  for (int j = 0; j < 200; j++){
	  get_angles();
	  // show Pitch_acc and Roll_acc on OLED
	  sprintf(temp[9], "P:%4.1f ", pitch_acc);
	  OLED_ShowString(0, 40, &temp[9]);  // show on OLED
	  sprintf(temp[9], "R:%4.1f", roll_acc); //
	  OLED_ShowString(70, 40, &temp[9]); // shown on OLED
	  OLED_Refresh_Gram();
	  HAL_Delay(10); // pause for 0.01 second
      }


  OLED_Clear();
  OLED_ShowString(0, 10, "Targets setting"); // show message on OLED display at line 40)
  OLED_ShowString(0, 20, "  procedure"); // show message on OLED display at line 40)
  OLED_ShowString(0, 35, "   starts"); // show message on OLED display at line 40)
  OLED_ShowString(0, 50, "..Keep still.."); // show message on OLED display at line 40)
  OLED_Refresh_Gram();


  set_targets(); //set the target  angles for the balance platform
  HAL_Delay(2000); // pause for 2 second to show message
  OLED_Clear();
}

float pitch_target, roll_target;  // target angles values for platform

void OLED_disp_msg3()
{  	// show Pitch_acc and Roll_acc on OLED
  	sprintf(temp[9], "P:%4.1f ", pitch_acc);
  	OLED_ShowString(0, 40, &temp[9]);  // show on OLED
	sprintf(temp[9], "R:%4.1f", roll_acc); //
  	OLED_ShowString(70, 40, &temp[9]); // shown on OLED

  	sprintf(temp[9], "T:%4.1f ", pitch_target);
  	OLED_ShowString(0, 50, &temp[9]);  // show on OLED
	sprintf(temp[9], "T:%4.1f", roll_target); //
  	OLED_ShowString(70, 50, &temp[9]); // shown on OLED

 	sprintf(temp[9], "dt: %5.3f", dt);  // cycle time
  	OLED_ShowString(0, 0, &temp[9]); // shown on OLED

  	OLED_Refresh_Gram(); // refresh OLED display
}


//***************************************************************************************

void Serial_tx(){
  	// send Platform  roll and pitch angles as roll and pitch targets to USB serial port
  	sprintf(sbuf[0], "%7.2f,", roll_KF); //
   	sprintf(sbuf[1], "%7.2f,", roll_target);
  	sprintf(sbuf[2], "%7.2f,", pitch_KF); //
  	sprintf(sbuf[3], "%7.2f\n\r", pitch_target); //yaw_gyro
  	for (int i=0; i<4; i++)// send the values to serial port: channel 15 to 17 on SerialPort
  	 	HAL_UART_Transmit(&huart3, sbuf[i], 8, HAL_MAX_DELAY); // send through serial port at 115200
    }


//***************************************************************************************

void Server_Motor12(){
	 // Front wheel server motor is controlled though PB14 and PB15
	 //   configured as Timer12 Channel 1 and Channel 2 PWM
	 // Pulse period = 20ms; : 1ms = 0 degree, 1.5 ms = 90 deg, 2 ms = 180 degree
	 // forward = 1.5 msec ?
	 // Timer3 frequency @ APB2 = 72MHz
	 // Required PWM frequency = 50Hz (= 20msec)
	 // Scaling required = 72000000/50= 1440K
	 //   set Prescaler to 720 and ARR = 2000 (see ioc setup)
	 // 1ms = 2000 * 1/20 = 100
	 // 2 msec = 200
	 //1.5 msec = 150

	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // start timer 12 for servor motor
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // start timer 12 for servor motor

  // initial position
	htim12.Instance->CCR1 = roll_servo_target;  // duty cycle is 1.5 ms
	htim12.Instance->CCR2 = pitch_servo_target;  // duty cycle is 1.5 ms
	HAL_Delay(2000);

}


//***************************************************************************************

int start=0;              // use to start stop the motor
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {// user push button interrupt
    // see EXTI0_IRQHandler() in stm32f4xx_it.c for interrupt
	if ( GPIO_Pin == USER_PB_Pin) {
		// toggle LED
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); // LED - A12
		if (start == 0)
			start = 1;
		else
			start = 0;
 	    }
}

//***************************************************************************************

//ring array to store data for moving average
#define window_size 10
float pitch_array[window_size] = {0}; // initialize all value to 0
float roll_array[window_size] = {0};

float mov_avr_pitch(float data)
{   static int index = 0;
    float sum;
    char *buff;


    pitch_array[index] = data;
    index++;
    if (index == window_size)
		index = 0;

	sum = 0;

	for (int i = 0; i<window_size; i++)
		sum = sum + pitch_array[i];

	return (sum/window_size);
}

//----------------------------------------------------------------------------------------

float mov_avr_roll(const float data)
{   static int index = 0;

    roll_array[index] = data;
    index++;
	if (index == window_size)
		index = 0;

	float sum = 0;
	for (int i = 0; i<window_size; i++)
		sum = sum + roll_array[i];

	return (sum/window_size);
}

//***************************************************************************************

#define data_size 500 // take most recent 500 values

void set_targets()  // use the average of 500 samples to set the platform target angles
{ float pitch_arr[data_size];
  float roll_arr[data_size];
  float pitch_sum = 0, roll_sum = 0;

  millisOld = HAL_GetTick();
  for (int i = 0; i < data_size; i++ ){
      get_angles();
      pitch_arr[i] = pitch_KF; // store entries into array
      roll_arr[i] = roll_KF;  // store entries i to array
      }
  // calculate average
  for (int i = 0; i < data_size; i++ ){
      pitch_sum = pitch_sum + pitch_arr[i];
      roll_sum = roll_sum + roll_arr[i];
      }
  pitch_target = pitch_sum/data_size;
  roll_target = roll_sum/data_size;
}

//***************************************************************************************

void get_angles()  //Calaculate pitch and toll angles using Kalman Filter
{
	  IMU_AccelRead(&imu); // read Accelerometer value
	  IMU_GyroRead(&imu); //  read Gyroscope value

	  // get the loop elapse time = sampling interval
	  millisNow = HAL_GetTick(); // store the current time for next round
	  dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
	  millisOld = millisNow; // store the current time for next round

	  // Calculate the angle based on Gyroscope x dt of 50msec
	  // pitch_gyro = pitch_gyro + imu.gyro[0] * dt; // in millisecond
	  // roll_gyro = roll_gyro + imu.gyro[1] * dt;
	  // yaw_gyro = yaw_gyro + imu.gyro[2] * dt;     // yaw - not needed

	   // convert the acc readings to pitch and roll angles
	   roll_acc=atan2(imu.acc[0],imu.acc[2]); //thetha = pitch = arcTan(aX/aZ)
	   roll_acc=-roll_acc*57.3; //change from radian to degree, change sign to match gyro's pitch
	   pitch_acc=atan2(imu.acc[1],imu.acc[2]); //Phi = roll = arcTan(aX/aZ)
	   pitch_acc=pitch_acc*57.3; //change from radian to degree//

	   // Complementary filter pitch AND ROLL
	   pitch_CF = 0.05*pitch_acc + 0.95*(pitch_CF + imu.gyro[0]*dt);	 // dt is msec
	   roll_CF = 0.05*roll_acc + 0.95*(roll_CF + imu.gyro[1]*dt);

	   //Calculate Kalman filter Pitch and Roll
	   // roll_gyro and pitch_gyro, roll_acc and pitch_acc alreadys calculated above
	   // calculate the roll and pitch estimated angles by gyro measurement
	   pitch_KF = pitch_KF + imu.gyro[0]*dt; // add the latest change in angles to previous values
	   roll_KF = roll_KF + imu.gyro[1]*dt;
	   // calculate the variance of the estimates by gyro
	   pitch_var_g = pitch_var_g + dt*dt*Var_gyro;
	   roll_var_g = roll_var_g + dt*dt*Var_gyro;
	   // calculate the Kalman Gain
	   KG_roll = roll_var_g/(roll_var_g + Var_acc);
	   KG_pitch = pitch_var_g/(pitch_var_g + Var_acc);
	   // update the gyro estimates using KG
	   roll_KF = roll_KF + KG_roll*(roll_acc-roll_KF);
	   pitch_KF = pitch_KF + KG_pitch*(pitch_acc-pitch_KF);
	   // update the gyro estimates variance using KG
	   roll_var_g = (1-KG_roll)*roll_var_g;
	   pitch_var_g = (1-KG_pitch)*pitch_var_g;
  }

//***************************************************************************************
// Control loop for Roll angle
float roll_angle;            // current Roll angle of platform
float roll_target;           // target Roll angle
float roll_error=0;          // error between target and actual
float roll_error_area = 0;   // area under error - to calculate I for PI implementation
float roll_error_old = 0;    // use these 3 variables to calculate D for PD/PID control
float roll_error_change = 0;
float roll_error_rate = 0;

void roll_PID(float angle, const float kp, const float ki, const float kd)
{   int roll_ServoVal;        // PWM servo value for roll servo motor
    const int ServoMax = 220; // maximum servor value
    const int ServoMin = 50;  // maximum servor value

    roll_error = roll_target - angle;

    /*millisNow = HAL_GetTick();
	dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
	millisOld = millisNow; // store the current time for next round*/

	//Ki
	roll_error_area = roll_error_area + roll_error*dt;
	//kd
	roll_error_change = roll_error - roll_error_old;
	roll_error_old = roll_error;
	roll_error_rate = (roll_error_change)/dt;

	//roll_ServoVal = (int)(roll_servo_target-(roll_error*kp)); // P control loop
	//roll_ServoVal = (int)(roll_servo_target-(roll_error*kp+roll_error_area*ki/10000));
	roll_ServoVal = (int)(roll_servo_target-(roll_error*kp+roll_error_area*ki + kd*roll_error_rate));


	if (roll_ServoVal > ServoMax)  // Clamp the PWM value to maximum value
	   roll_ServoVal = ServoMax;
	if (roll_ServoVal < ServoMin)  // Clamp the PWM value to minimum value
	   roll_ServoVal = ServoMin;

	//roll_ServoVal = (int)(roll_ServoVal-(roll_error_rate*kd));

	htim12.Instance->CCR1 = roll_ServoVal;      // send PWM duty cycle to servo motor

	sprintf(temp[9], "rS:%3d", roll_ServoVal);  // show Servo PWM value on OLED display
	OLED_ShowString(70, 20, &temp[9]);
	OLED_Refresh_Gram();
}


//---------------------------------------------------------------------------------
// Control loop for Pitch angle
float pitch_angle;            // current pitch angle of platform
float pitch_target;           // target angle
float pitch_error = 0;          // error between target and actual
float pitch_error_area = 0;   // area under error - to calculate I for PI implementation
float pitch_error_old = 0, pitch_error_change = 0, pitch_error_rate = 0; // to calculate D for PID control

void pitch_PID(float angle, const float kp, const float ki, const float kd)
{   int pitch_ServoVal;       // PWM servo value for Pitch servo motor
    const int ServoMax = 220; // maximum servor value
    const int ServoMin = 50;  // maximum servor value

    //return; // disable this function - remove this line to implement Pitch control loop

    //pitch_ServoVal = (int)(pitch_servo_target-(pitch_error*kp));
    pitch_error = pitch_target - angle;

	/*millisNow = HAL_GetTick();
	dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
	millisOld = millisNow; // store the current time for next round*/

	//Ki
	pitch_error_area = pitch_error_area + pitch_error*dt;
	//kd
	pitch_error_change = pitch_error - pitch_error_old;
	pitch_error_old = pitch_error;
	pitch_error_rate = (pitch_error_change)/dt;
	pitch_ServoVal = (int)(pitch_servo_target-(pitch_error*kp+pitch_error_area*ki + (kd*pitch_error_rate)));

    if (pitch_ServoVal > ServoMax)  // Clamp the PWM value to maximum value
	   pitch_ServoVal = ServoMax;
	if (pitch_ServoVal < ServoMin)  // Clamp the PWM value to minimum value
	   pitch_ServoVal = ServoMin;

	htim12.Instance->CCR2 = pitch_ServoVal;  // send duty cycle to Pitch servo motor

	sprintf(temp[9], "pS:%3d", pitch_ServoVal);  // show Servo PWM value on OLED display
	OLED_ShowString(0, 20, &temp[9]);
	OLED_Refresh_Gram(); // refresh OLED display
}

//***************************************************************************************

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
     float Kp = 0;  // PID parameters
     float Kd = 0;
     float Ki = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
     OLED_disp_msg1();  // display startup message on OLED

     // Initialize the IMU
     char *msg;
     int i = IMU_Initialise(&imu, &hi2c2, &huart3);
     if (i == 0) // 0 error (i.e. no error)
  	    msg = "IMU Init OK   \r\n";
     else
        msg = "IMU Init Error\r\n";
     HAL_UART_Transmit(&huart3, msg, 16, HAL_MAX_DELAY);  // send message to serial port

     OLED_disp_msg2();  // display message2 on OLED to setup platform

     roll_servo_target = 150;   // nominal PWM value for roll servo motor
     pitch_servo_target = 65;   // nominal PWM value for pitch servo motor

     millisOld = HAL_GetTick(); // get first time value before starting - for Gyro and PID dt computation
     start = 1;  // for user push button detection
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
     //------------------------------------------------------------------------------------
     while (start == 1){ //do until User button is pressed
	   get_angles();     //get roll and pitch angles from IMU

	   //filtering the angles using FIR moving average filter
	   roll_angle = mov_avr_roll(roll_KF);
	   pitch_angle = mov_avr_pitch(pitch_KF);

	   //2 1.5 0.25 | 1.5 1.5 0.25
	   // Execute control loop for Roll angle with its PID parameters
	   Kp = 2;     // P
	   Ki = 1.5;     // I
	   Kd = 0.25;     // D
	   roll_PID(roll_angle, Kp, Ki, Kd);

	   // Execute control loop for Pitch angle with its PID parameters
	   Kp = 1.5;     // P
	   Ki = 1.5;     // I
	   Kd = 0.25;     // D
	   pitch_PID(pitch_angle, Kp, Ki, Kd);

       Serial_tx(); //send roll & pitch angles, roll & pitch targets to serial port

  	   OLED_disp_msg3(); //display information on OLED

  	   HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); // toggle LED for heart beat indicator
     } // while(start == 1)

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
     //while loop exit - user push button detected
     OLED_Clear();
     OLED_ShowString(0, 10, "Program Stop"); // show message on OLED display at line 10)
     OLED_ShowString(0, 30, "Press Reset");  // show message on OLED display at line 30)
     OLED_ShowString(0, 40, "button to");    // show message on OLED display at line 40)
     OLED_ShowString(0, 50, "restart");      // show message on OLED display at line 50)
     OLED_Refresh_Gram();
     while(1); // stop - do nothing
  /* USER CODE END 3 */
} //main

//**************************************************************************************************

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 720;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 2000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin RESET__Pin SDIN_Pin SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RESET__Pin|SDIN_Pin|SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED3_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/*
void OLED_show(void *argument, int y, int x) // display message on OLED panel
{
	//uint8_t hello[20]="Hello World";
	OLED_Init();
	OLED_Display_On();
//	OLED_ShowString(10,10,argument);
	OLED_ShowString(y, x, argument);
	OLED_Refresh_Gram();
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
