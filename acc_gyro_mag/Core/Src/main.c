/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDR 0x68 << 1  //acc gyro addr
#define MAG_ADDR 0x0C <<1 //Magnetometer I2C address
#define CONFIG_REG 0x1A
#define ACC_CTRL_REG 0x1C
#define GYRO_CTRL_REG 0x1B
#define ACC_CTRL_REG2 0x1D
#define INT_PIN_CFG 0x37
#define USER_CTRL 0x6A
#define AK8963_ADDR 0x0C << 1
#define AK8963_CNTL1 0x0A

// MPU9250 registers
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_MAG_XOUT_L 0x03

#define NUM_SAMPLES 2000
#define GYRO_NUM_SAMPLES 2000
#define MAG_NUM_SAMPLES 1000
#define GYRO_SENSITIVITY 65.5f //for +/- 500dps setting
#define ACCEL_SENSITIVITY 4096.0f // For ±8g setting
#define DT 0.004 //4ms update period for gyro integration
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float accx_offset, accy_offset, accz_offset;
float gyrox_offset, gyroy_offset, gyroz_offset;
float magx_offset, magy_offset, magz_offset;
float rad_to_deg = (180/(22/7));
float pitch = 0, roll = 0, yaw = 0;
float gyrPitch=0, gyrRoll=0, gyrYaw=0;
float Mx, My; //variables for magneto-meter readings corrected for tilt
float Kalman1DOutput[] = {0,0};
float KalmanAngleRoll = 0, KalmanAnglePitch = 0, KalmanAngleYaw = 0;
float KalmanUncertaintyRoll = 0, KalmanUncertaintyPitch = 0, KalmanUncertaintyYaw = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU9250_Init(void);
void MPU9250_Read_Accel(int16_t *accelData);
void MPU9250_Read_Gyro(int16_t *gyroData);
void MPU9250_Read_Mag(int16_t *magData);
void I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size);
void I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size);
void MPU9250_Calibrate_acc();
void processSensorData(int16_t accRaw[3], int16_t gyroRaw[3]);
void MPU9250_Calibrate_gyro(float* gyroOffsetX, float* gyroOffsetY, float* gyroOffsetZ);
void MPU9250_Calibrate_mag(float * magOffsetX, float *magOffsetY, float *magOffsetZ);
float convertGyro(int16_t raw);
float convertAccel(int16_t raw);
void kalman_1d(float KalmanState, float KalmanUncertainty, float RotationRate, float Accel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

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
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t data = 0x00;
	I2C_Write(MPU9250_ADDR, MPU9250_PWR_MGMT_1, &data, 1);
	MPU9250_Init();
	MPU9250_Calibrate_acc();
	MPU9250_Calibrate_gyro(&gyrox_offset, &gyroy_offset, &gyroz_offset);
	MPU9250_Calibrate_mag(&magx_offset, &magy_offset, &magz_offset);

	int16_t accelData[3], gyroData[3], magData[3];
	float accgX, accgY, accgZ, gyroX, gyroY, gyroZ, magX, magY, magZ;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		MPU9250_Read_Accel(accelData);
		MPU9250_Read_Gyro(gyroData);
		MPU9250_Read_Mag(magData);
		//processSensorData(accelData, gyroData);
		//MPU9250_Read_Mag(magData);
		accgX = (float)(((accelData[0]/4096.0f) - accx_offset) * 9.80665);
		accgY = (float)(((accelData[1]/4096.0f) - accy_offset) * 9.80665);
		accgZ = (float)(((accelData[2]/4096.0f) - accz_offset) * 9.80665);

		gyroX = (float)((gyroData[0]/GYRO_SENSITIVITY)-gyrox_offset);
		gyroY = (float)((gyroData[1]/GYRO_SENSITIVITY)-gyroy_offset);
		gyroZ = (float)((gyroData[2]/GYRO_SENSITIVITY)-gyroz_offset);

		magX = ((float)magData[0] - magx_offset) * 1.0f * 0.15f;// 0.15 µT per LSB for 16-bit mode
		magY = ((float)magData[1] - magy_offset) * 1.0f * 0.15f;
		magZ = ((float)magData[2] - magz_offset) * 1.0f * 0.15f;

		//printf("Ax: %f m/ss Ay: %f m/ss, Az: %f m/ss \n", accgX, accgY, accgZ);
		//printf("Gx: %f dps Gy: %f dps Gz: %f dps \n", gyroX, gyroY, gyroZ);
		//roll = atan2(accgY, accgZ) * rad_to_deg;
		roll = atan2(accgY, sqrt(accgX*accgX + accgZ*accgZ)) * rad_to_deg;
		pitch = atan2(-accgX, sqrt(accgY*accgY + accgZ*accgZ)) * rad_to_deg;
		Mx = magX*cos(pitch) + magZ*sin(pitch);
		My = magX*sin(roll)*sin(pitch) + magY*cos(roll) - magZ*sin(roll)*cos(pitch);
		yaw = atan2(-My, Mx) * rad_to_deg;//calculate yaw angle with tilt compensation
		if(yaw < 0)
			yaw += 360;//ensure readings are between 0 and 360 degrees

		kalman_1d(KalmanAngleRoll, KalmanUncertaintyRoll, gyroX, roll);
		KalmanAngleRoll = Kalman1DOutput[0];
		KalmanUncertaintyRoll = Kalman1DOutput[1];

		kalman_1d(KalmanAnglePitch, KalmanUncertaintyPitch, gyroY, pitch);
		KalmanAnglePitch = Kalman1DOutput[0];
		KalmanUncertaintyPitch = Kalman1DOutput[1];

		kalman_1d(KalmanAngleYaw, KalmanUncertaintyYaw, gyroZ, yaw);
		KalmanAngleYaw = Kalman1DOutput[0];
		KalmanUncertaintyYaw = Kalman1DOutput[1];

		printf("Kalman -> KRoll: %f deg Kpitch: %f deg Kyaw: %f deg\n", KalmanAngleRoll, KalmanAnglePitch, KalmanAngleYaw);
		/*gyrPitch += (gyroY * DT);
		gyrRoll += (gyroX * DT);
		gyrYaw += (gyroZ * DT);
		printf("Acc -> AccRoll: %f deg  AccPitch: %f deg AccYaw: %f deg \n", roll, pitch, yaw);
		printf("Gyro -> GRoll: %f deg  GPitch: %f deg GYaw: %f deg \n", gyrRoll, gyrPitch, gyrYaw);
		//printf("Mag -> X: %f uT Y: %f uT Z: %f uT \n", magX, magY, magZ);*/
		HAL_Delay(4); //4ms loop
		/*printf("Ax: %f Ay: %f, Az: %f | Gx: %f Gy: %f Gz: %f\n",
			  accelData[0], accelData[1], accelData[3], gyroData[0], gyroData[1], gyroData[2]);

	  printf("Mx: %f My: %f Mz: %f\n", magData[0], magData[1], magData[2]);
	  HAL_Delay(1000);*/
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	huart2.Init.BaudRate = 115200;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MPU9250_Init(void) {
	uint8_t check, data;
	I2C_Read(MPU9250_ADDR, MPU9250_WHO_AM_I, &check, 1);
	if (check == 0x71) {
		printf("MPU found\n");
		data = 0x00;
		I2C_Write(MPU9250_ADDR, MPU9250_PWR_MGMT_1, &data, 1);
	}
	else{
		printf("No device found\n");
	}

	data = 0x10;
	I2C_Write(MPU9250_ADDR, ACC_CTRL_REG, &data, 1); //set 8g full scale for accelerometer
	data = 0x08;
	I2C_Write(MPU9250_ADDR, GYRO_CTRL_REG, &data, 1); //set 500dps full scale for gyro
	data = 0x05;
	I2C_Write(MPU9250_ADDR, ACC_CTRL_REG2, &data, 1); //set 10Hz (former 5) low pass filter for accel
	data = 0x05;
	I2C_Write(MPU9250_ADDR, CONFIG_REG, &data, 1); //set 10Hz low pass filter for gyro
	data = 0x00;
	I2C_Write(MPU9250_ADDR, USER_CTRL, &data, 1); // Disable I2C master mode to allow bypass
	data = 0x02;
	I2C_Write(MPU9250_ADDR, INT_PIN_CFG, &data, 1); //enable bypass to acces magnetometer
	data = 0x16;
	data = 0x01;
	I2C_Write(AK8963_ADDR, 0x0B, &data, 1); // Reset AK8963 (PWR_MGMT)
	HAL_Delay(100);  // Wait for reset
	data = 0x16;
	I2C_Write(AK8963_ADDR, AK8963_CNTL1, &data, 1); // Continuous mode 2 (100Hz, 16-bit output)
	I2C_Read(AK8963_ADDR, AK8963_CNTL1, &check, 1);
	printf("CNTL1 Register: 0x%02X\n", check);
	check = 0;
	data = 0;
	I2C_Read(AK8963_ADDR, 0x00, &check, 1);//read WIA register, should return 0x48
	if(check == 0x48)
		printf("Magnetometer found \n");
	else
		printf("No Magnetometer found. WIA: %d \n", check);
}

void MPU9250_Calibrate_acc()
{
	int16_t accelData[3];

	for(int i=0; i<NUM_SAMPLES; i++)
	{
		MPU9250_Read_Accel(accelData);
		accx_offset += (float)(accelData[0]/4096.0f);
		accy_offset += (float)(accelData[1]/4096.0f);
		accz_offset += (float)(accelData[2]/4096.0f);
		//HAL_Delay(10);

	}

	accx_offset /= NUM_SAMPLES;
	accy_offset /= NUM_SAMPLES;
	accz_offset = (accz_offset/NUM_SAMPLES) - 1.0f;

	printf("Accel Calibration Done. Offsets are: X: %f, Y: %f, Z: %f\n", accx_offset, accy_offset, accz_offset);
}

void MPU9250_Calibrate_gyro(float* gyroOffsetX, float* gyroOffsetY, float* gyroOffsetZ)
{
	int32_t sumX=0, sumY=0, sumZ=0;
	int16_t gyroData[3];

	for(int i=0; i<GYRO_NUM_SAMPLES; i++)
	{
		MPU9250_Read_Gyro(gyroData);
		/*sumX += gyroData[0];
		sumY += gyroData[1];
		sumZ += gyroData[2];*/
		*gyroOffsetX += (float)(gyroData[0]/GYRO_SENSITIVITY);
		*gyroOffsetY += (float)(gyroData[1]/GYRO_SENSITIVITY);
		*gyroOffsetZ += (float)(gyroData[2]/GYRO_SENSITIVITY);
	}

	*gyroOffsetX /= GYRO_NUM_SAMPLES;
	*gyroOffsetY /= GYRO_NUM_SAMPLES;
	*gyroOffsetZ /= GYRO_NUM_SAMPLES;

	printf("Gyro calibration Done. Offsets dps -> Gx: %f Gy: %f Gz: %f \n",
			*gyroOffsetX, *gyroOffsetY, *gyroOffsetZ);
}

void MPU9250_Calibrate_mag(float * magOffsetX, float *magOffsetY, float *magOffsetZ)
{
	int16_t magdata[3];
	int16_t magX_min = 32767, magX_max = -32768;
	int16_t magY_min = 32767, magY_max = -32768;
	int16_t magZ_min = 32767, magZ_max = -32768;

	printf("Rotate the sensor in all directions for calibration ...\n");

	for(int i=0; i<MAG_NUM_SAMPLES; i++)
	{
		MPU9250_Read_Mag(magdata);

		if(magdata[0] < magX_min) magX_min = magdata[0];
		if(magdata[0] > magX_max) magX_max = magdata[0];

		if(magdata[1] < magY_min) magY_min = magdata[1];
		if(magdata[1] > magY_max) magY_max = magdata[1];

		if(magdata[2] < magZ_min) magZ_min = magdata[2];
		if(magdata[2] > magZ_max) magZ_max = magdata[2];

		HAL_Delay(10);
	}

	//Offsets for hard iron correction
	*magOffsetX = ((int32_t)magX_max + (int32_t)magX_min)/2.0;//prromote to 32 bit
	*magOffsetY = ((int32_t)magY_max + (int32_t)magY_min)/2.0;//to prevent overflow
	*magOffsetZ = ((int32_t)magZ_max + (int32_t)magZ_min)/2.0;

	printf("Mag calibration done. Offsets -> Mx: %f My: %f Mz: %f \n",
			*magOffsetX, *magOffsetY, *magOffsetZ);
}


void MPU9250_Read_Accel(int16_t *accelData) {
	uint8_t rawData[6];
	I2C_Read(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, rawData, 6);
	accelData[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
	accelData[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
	accelData[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void MPU9250_Read_Gyro(int16_t *gyroData) {
	uint8_t rawData[6];
	I2C_Read(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, rawData, 6);
	gyroData[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
	gyroData[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
	gyroData[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void MPU9250_Read_Mag(int16_t *magData) {
	uint8_t rawData[6];
	uint8_t status;
	uint8_t overflowCheck;

	I2C_Read(AK8963_ADDR, 0x02, &status, 1); // Read ST1 (Data Ready Status)
	if (!(status & 0x01)) {
		printf("No new magnetometer data available\n");
		return;  // Exit if no new data
	}
	I2C_Read(AK8963_ADDR, 0x03, rawData, 6);//read raw data from x low register
	magData[0] = (int16_t)((rawData[1] << 8) | rawData[0]); //mag x
	magData[1] = (int16_t)((rawData[3] << 8) | rawData[2]); //mag y
	magData[2] = (int16_t)((rawData[5] << 8) | rawData[4]); //mag z

	I2C_Read(AK8963_ADDR, 0x09, &overflowCheck, 1);
	if (overflowCheck & 0x08) {
		printf("Magnetometer overflow detected\n");
		return;  // Data is invalid, so discard it
	}
}

void I2C_Read(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size) {
	HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
}

void I2C_Write(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t size) {
	HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
}

float convertAccel(int16_t raw) {
	return (float) ((raw / ACCEL_SENSITIVITY) * 9.80665);
}

float convertGyro(int16_t raw)
{
	return (float)raw/GYRO_SENSITIVITY;
}

void processSensorData(int16_t accRaw[3], int16_t gyroRaw[3])
{
	float accel[3], gyro[3];

	//Convert accel into m/s^2
	for(int i=0; i<3; i++)
		accel[i] = convertAccel(accRaw[i]);

	for(int i=0; i<3; i++)
		gyro[i] = convertGyro(gyroRaw[i]);

	printf("Accel -> X: %f m/ss Y: %f m/ss Z: %f m/ss\n", accel[0], accel[1], accel[2]);
	printf("Gyro -> X: %f dps Y: %f dps Z: %f dps\n", gyro[0], gyro[1], gyro[2]);
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float RotationRate, float Accel)
{
	KalmanState = KalmanState + 0.004*RotationRate;
	KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
	float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty+ 3*3);
	KalmanState = KalmanState + KalmanGain * (Accel - KalmanState); //update
	KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

	Kalman1DOutput[0] = KalmanState;
	Kalman1DOutput[1] = KalmanUncertainty;
}

int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
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
