/* @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#define FALSE 0
#define ERROR 0
#define TRUE 1
#define GOOD 1
#define NCMDS 10
#define FALSE 0
#define TRUE 1
#define ESC 27
#define BCKSP 32
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define VERSION "v1.0 grupo 2 LPI-II DEIC UM2018"
#define PORT_NUM 4
#define PORT_INCREMENT  0x0400U
#define GPIO_8_PINS  GPIO_PIN_All & 0x00FFU
#define DIGITAL_OUT_N_PINS
#define MAX_32_BIT 0xffffffffu
#define MAX_16_BIT 0xffffu
#define MAX_8_BIT 0xffu

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct str_arr
{
	char string_array[5][40];
	size_t size;
} str_arr;

uint8_t Rx_index=0;
uint8_t Rx_Buffer[128]={0};
volatile uint8_t Received_data=0;
volatile _Bool cmd_flag = FALSE;
uint8_t command_buffer[128] = {0};
uint8_t last_command_buffer[128] = {0};
typedef struct { char * command; _Bool(*command_function)(str_arr*); } t_cmdstruct;
typedef struct { char string_array[5][40]; size_t size; } t_strstruct;
volatile uint8_t Tx_tail=0;
volatile uint8_t Tx_head=0;
uint8_t Tx_Buffer[TX_BUFFER_SIZE]={0};
volatile _Bool Rx_active_flag = TRUE;
volatile _Bool Rx_buf_full = FALSE;
int asd = 0;
int *ptr = &asd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void processCommand(void);
static void resetFlags(void);
static void readRxBuffer(void);
static _Bool commandSearch(char*);
void showErrorMessage(void);
size_t strlen(const char *s);

void send_MI_msg(uint32_t addr,uint16_t pin_setting);
void send_MO_msg(uint32_t addr,uint16_t pin_setting);
void send_MR_msg(uint32_t addr,uint32_t value);
void send_MW_msg(void);
void send_RD_msg(uint32_t addr,uint16_t pin_setting,uint32_t value);
void send_WD_msg(void);
_Bool memoryRead(str_arr*);
	_Bool memoryWrite(str_arr*);
_Bool	makePortPinInput(str_arr*);
_Bool makePortPinOutput(str_arr*);
_Bool  readDigitalInput(str_arr*);
_Bool  writeDigitalOutput(str_arr*);
_Bool  analogRead(str_arr*arr){return TRUE;}
_Bool  writeE2PROM(str_arr*arr){return TRUE;}
_Bool  readE2PROM(str_arr* arr){return TRUE;}
_Bool  sendHelp(str_arr*arr){return TRUE;}
_Bool  sendVersion(str_arr*arr){return TRUE;}
_Bool convert_str_hexa_uint32_t(uint32_t* value,char * addr);
_Bool convert_str_hexa_uint8_t(uint8_t* value,char * addr);
_Bool convert_str_hexa_uint16_t(uint16_t* value,char* addr);
static void set_msg(char* msg,uint32_t addr,uint32_t *memory);
static void put_buffer(char*ch);

static _Bool GPIO_clk_enable(char port_name);
static char check_GPIO_Port_Addr(uint32_t* addr);
static void GPIO_config(uint32_t* port_addr, uint32_t mode,uint16_t pin_setting);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static t_cmdstruct cmd_table[] = {
	{"MR", memoryRead},
	{"MW", memoryWrite},
	{"MI", makePortPinInput},
	{"MO", makePortPinOutput},
	{"RD", readDigitalInput},
	{"WD", writeDigitalOutput},
	{"RA", analogRead},
	{"WE", writeE2PROM},
	{"RE", readE2PROM},
	{"?", sendHelp},
	{"VER", sendVersion}
};
static t_strstruct str_struct;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
str_arr x;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	asd = 0;

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	str_struct.size = 0;
	unsigned int *ptr = 	(unsigned int *)GPIOA_BASE;
	printf("Ready for command\n>");
	/* Activate the received interrupt for one char*/
	HAL_UART_Receive_IT(&huart3, &Rx_Buffer[Rx_index], 1);
	printf("Ready for command\n>");
	strcpy(x.string_array[0],"MW");
	strcpy(x.string_array[1],"40020000");
	strcpy(x.string_array[2],"4");
	x.size =3;
	Rx_index=0;
  /* USER CODE END 2 */
makePortPinInput(&x);
readDigitalInput(&x);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(cmd_flag == TRUE){
			processCommand();
			if(commandSearch(str_struct.string_array[0]) == ERROR) 
				showErrorMessage();
			resetFlags();
		}

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);
	
}

/* USER CODE BEGIN 4 */
void resetFlags(){
	printf("\n>");
	cmd_flag = 0;
	//flag_receive = 1;
}
void showErrorMessage(){
	printf("Invalid command! Check syntax or try a different one\n");
}
void readRxBuffer(){
	strcpy((char*)command_buffer,(char*)Rx_Buffer);
	strcpy((char*)last_command_buffer,(char*)Rx_Buffer);
}
//Process the commands (parsing)
void processCommand(){
	char index = 0;
	char * token = NULL;
	readRxBuffer();
	
	//start parsing
	token = strtok((char*)command_buffer," .");
	while(token != NULL){
		strcpy(str_struct.string_array[index],token);
		index = index + 1;
		token = strtok(NULL," .");	
	}
	str_struct.size = index;
}

_Bool commandSearch(char * new_command){
	size_t index;
	str_arr arr;
	for(index = 0; index <= NCMDS; index++){
		if(strcmp(cmd_table[index].command, new_command) == 0){
			cmd_table[index].command_function(&arr);
			return GOOD;
		}
	}
	return ERROR;
}

_Bool  memoryRead(str_arr* arr)
{
	uint32_t addr;
	uint8_t length;
	uint32_t *memory;
	size_t index;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE ||convert_str_hexa_uint8_t(&length,(*arr).string_array[2]) == FALSE)
		return FALSE;
	else
	{
		for(index = 0; index <length ;index ++)
		{
		memory = (uint32_t*)addr;
		send_MR_msg(addr,*memory);
		++memory;
		++addr;
		}
	}
	return TRUE;
}

_Bool  memoryWrite(str_arr* arr)
{
	uint32_t addr;
	uint8_t length;
	uint32_t data;
	uint32_t *memory;
	size_t index;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE||convert_str_hexa_uint8_t(&length,(*arr).string_array[2]) == FALSE|| convert_str_hexa_uint32_t(&data,(*arr).string_array[3]) == FALSE)
		return FALSE;
	else
	{
		for(index = 0; index <length ;index ++)
		{
			memory = (uint32_t*)addr;
		*memory = data;
		++memory;
		++addr;
		}
	}
	send_MW_msg();
	return TRUE;
}

_Bool  makePortPinInput(str_arr* arr)
{
	uint32_t addr;
	uint16_t pin_set;
	char port_name;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE|| convert_str_hexa_uint16_t(&pin_set,(*arr).string_array[2]) == FALSE)
		return FALSE;
	else
	{
		port_name = check_GPIO_Port_Addr((uint32_t*) addr);
		GPIO_clk_enable(port_name);
		GPIO_config((uint32_t*) addr,GPIO_MODE_INPUT,pin_set);
		send_MI_msg(addr,pin_set);
	}
	return TRUE;
}

_Bool  makePortPinOutput(str_arr* arr)
{
	uint32_t addr;
	uint16_t pin_set;
	char port_name;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE|| convert_str_hexa_uint16_t(&pin_set,(*arr).string_array[2]) == FALSE)
		return FALSE;
	else
	{
		port_name = check_GPIO_Port_Addr((uint32_t*) addr);
		GPIO_clk_enable(port_name);
		GPIO_config((uint32_t*) addr,GPIO_MODE_OUTPUT_PP,pin_set);
		send_MO_msg(addr,pin_set);
	}
	return TRUE;
}

_Bool  readDigitalInput(str_arr* arr)
{
	uint32_t addr;
	uint8_t pin_set;
	GPIO_TypeDef* GPIO_x;
	char port_name;
	uint8_t data=0;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE|| convert_str_hexa_uint8_t(&pin_set,(*arr).string_array[2]) == FALSE)
		return FALSE;
	else
	{
		GPIO_x = (GPIO_TypeDef*)(addr);
		port_name = check_GPIO_Port_Addr((uint32_t*) addr);
		GPIO_clk_enable(port_name);
		GPIO_config((uint32_t*) addr,GPIO_MODE_INPUT,pin_set);
		data = ((*GPIO_x).IDR &( ~(0xffffff00)));
		send_RD_msg(addr,pin_set,data);
	}
	return TRUE;
}

_Bool  writeDigitalOutput(str_arr* arr)
{
	uint32_t addr;
	uint8_t pin_set;
	GPIO_TypeDef* GPIO_x;
	char port_name;
	
	if((*arr).size != 3 || convert_str_hexa_uint32_t(&addr,(*arr).string_array[1]) == FALSE|| convert_str_hexa_uint8_t(&pin_set,(*arr).string_array[2]) == FALSE)
		return FALSE;
	else
	{
		GPIO_x = (GPIO_TypeDef*)(addr);
		port_name = check_GPIO_Port_Addr((uint32_t*) addr);
		GPIO_clk_enable(port_name);
		GPIO_config((uint32_t*) addr,GPIO_MODE_OUTPUT_PP,GPIO_8_PINS);
		(*GPIO_x).ODR ^= (pin_set|0xffffff00);
		send_WD_msg();
	}
	return TRUE;
}

void send_MI_msg(uint32_t addr,uint16_t pin_setting)
{
	char port_name;
	char str_value[9];
	char msg[128];
	size_t index;
	uint16_t mask = 1;
	for(index=0; index <= 8; index++)
	{
		if(((pin_setting&mask)>>index)== 1)
			sprintf(str_value,"%d ",index);
		mask = mask << 1;
	}
	port_name = check_GPIO_Port_Addr((uint32_t*) addr);
	sprintf(msg,"Port %c pins %s set as Input Mode\r\n",port_name ,str_value);
	put_buffer(msg);
}

void send_MO_msg(uint32_t addr,uint16_t pin_setting)
{
	char port_name;
	char str_value[9];
	char msg[128];
	size_t index;
	uint16_t mask = 1;
	for(index=0; index <= 8; index++)
	{
		if((pin_setting&mask)== 1)
			sprintf(str_value,"%d ",index);
	}
	port_name = check_GPIO_Port_Addr((uint32_t*) addr);
	sprintf(msg,"Port %c pins %s set as Output Mode\r\n",port_name ,str_value);
	put_buffer(msg);
}

void send_MR_msg(uint32_t addr,uint32_t value)
{
	char str_addr[9];
	char str_value[9];
	char msg[128];
	
	sprintf(str_value, "%x",value);
	sprintf(str_addr, "%x", addr);
	sprintf(msg,"Adress %s has the value %s\r\n",str_addr ,str_value);
	put_buffer(msg);
}

void send_MW_msg()
{
	put_buffer("Written value.");
}

void send_RD_msg(uint32_t addr,uint16_t pin_setting, uint32_t value)
{
	char port_name;
	char msg[128];
	size_t index;
	uint16_t mask = 1;
	port_name = check_GPIO_Port_Addr((uint32_t*) addr);
	sprintf(msg,"Port %c\r\n",port_name);
	put_buffer(msg);
	for(index=0; index < 8; index++)
	{
		if(((pin_setting&mask)>>index)== 1)
		{
			sprintf(msg,"Pin %d: value:%d\r\n",index ,((value&pin_setting&mask)>>index));
			put_buffer(msg);
		}
		mask = mask << 1;
	}
}

void send_WD_msg()
{
	put_buffer("Written digital value.");
}






void send_RA_msg()
{
	
}

// check if addr is a port addr
static char check_GPIO_Port_Addr(uint32_t* addr)
{
	size_t index;
	char port_name ='A';
	uint32_t gpio = GPIOA_BASE; // starts in port A
	for(index=0;index < PORT_NUM; index++) 
	{
		if((uint32_t)addr == gpio)
			return port_name;
		gpio += PORT_INCREMENT;		// increments port pointer to other ports addr
		port_name +=1;
	}
	return NULL;
}

static _Bool GPIO_clk_enable(char port_name)
{
	switch(port_name)
	{
		case'A':
			 __HAL_RCC_GPIOA_CLK_ENABLE();
			break;
		case'B':
			 __HAL_RCC_GPIOB_CLK_ENABLE();
			break;
		case'C':
			 __HAL_RCC_GPIOC_CLK_ENABLE();
			break;
		case'D':
			 __HAL_RCC_GPIOD_CLK_ENABLE();
			break;
		case'E':
			 __HAL_RCC_GPIOE_CLK_ENABLE();
			break;
		case'F':
			 __HAL_RCC_GPIOF_CLK_ENABLE();
			break;
		case'G':
			 __HAL_RCC_GPIOG_CLK_ENABLE();
			break;
		default:
			return FALSE;
	}
	return TRUE;
}

static void GPIO_config(uint32_t* port_addr, uint32_t mode, uint16_t pin_setting)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin_setting;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	if(mode != GPIO_MODE_INPUT)
	{
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_WritePin((GPIO_TypeDef*)((uint32_t*) port_addr),pin_setting, GPIO_PIN_RESET);
	}
	
  HAL_GPIO_Init((GPIO_TypeDef*)((uint32_t*) port_addr), &GPIO_InitStruct);
}

void  HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t aux_ch = 0;
	
	if((*huart).Instance == USART3 && Rx_active_flag == TRUE)
	{
		aux_ch = Rx_Buffer[Rx_index];
		switch(aux_ch)
		{
			case(0x0D):
				Rx_Buffer[Rx_index]='\0';
				Rx_active_flag = FALSE;
				cmd_flag = TRUE;
				break;
			case (ESC):
				Rx_index = 0;
			case (BCKSP):
				break;
			default:
				if(Rx_index != (RX_BUFFER_SIZE-1)) 
				Rx_index = Rx_index + 1; // increments index
		}	
		HAL_UART_Receive_IT(&huart3,&Rx_Buffer[Rx_index],1); // prepares to receive another char
	}
}

// transmission interrupt function
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if((*huart).Instance == USART3)
	{
			if(Tx_head!=Tx_tail)
			{
				Tx_head++;
				Tx_head &=(TX_BUFFER_SIZE-1);
				if(Tx_Buffer[Tx_head] != '\0')
				HAL_UART_Transmit_IT(&huart3,&Tx_Buffer[Tx_head],1);
			}
	}
}


// put Tx_Buffer function
static void put_buffer(char ch[])
{
	int index;
	size_t ch_len = strlen(ch);
	for(index=0; index < ch_len ;index++)
	{
		Tx_Buffer[Tx_tail] = *ch; 
		ch++;
		Tx_tail++;
		Tx_tail&=(TX_BUFFER_SIZE-1); 
		while(Tx_tail==Tx_head); // wait while Tx_buffer full
	}
	if(HAL_UART_GetState(&huart3)!=HAL_BUSY) // checks if it is already transmitting
		HAL_UART_Transmit_IT(&huart3,&Tx_Buffer[Tx_head],1);// initiates transmission
}

size_t strlen(const char *s)
{
	const char *ss = s;
	size_t num = 0;
	while(*ss)
	{
		ss++;
		num++;
	}
	return num;
}

int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,100);
	return ch;
}

_Bool convert_str_hexa_uint32_t(uint32_t* value,char* addr)
{
	int aux =0;
	int i = 0;
	i = sscanf(addr, "%x", &aux);
	if(i == FALSE || ((unsigned int)aux) > MAX_32_BIT)
	{
		return FALSE;
	}
	else
	{
		*value = aux;
		return TRUE;
	}
}

_Bool convert_str_hexa_uint16_t(uint16_t* value,char* addr)
{
	uint32_t aux = *value;
	if(convert_str_hexa_uint32_t(&aux,addr) && aux < MAX_16_BIT)
	{
		*value = aux;
		return TRUE;
	}
	return FALSE;
}

_Bool convert_str_hexa_uint8_t(uint8_t* value,char * addr)
{
	uint32_t aux = *value;
	if(convert_str_hexa_uint32_t(&aux,addr) && aux < MAX_8_BIT)
	{
		*value = aux;
		return TRUE;
	}
	return FALSE;
}

/* USER CODE END 4 */

	/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
