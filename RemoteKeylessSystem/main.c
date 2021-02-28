/*************************************************
STM32F4 Discovery - Keyless Remote System with Rolling Code through C
Gebze Technical University - Electronics Engineering Dept.
Authors: Ahmet Hamdi Coruk & Ali Sacid Karadogan
ELEC458 Course Project Team 1
*************************************************/

#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "aes.h"
#include "string.h"

/*************************************************
Definitions
*************************************************/
//Common
#define FALLING_EDGE 0
#define RISING_EDGE 1
#define SINGLE_PRESS 1
#define LONG_PRESS_TIME 1500
#define DOUBLE_PRESS 2
#define SRC_ADDRESS 0x24
#define DEST_ADDRESS 0x56
#define ECB 1
#define S2_ADDRESS 0x08008000
#define S3_ADDRESS 0x0800C000
#define SUCCESS 0
#define FAIL 1
#define AF7 0x07

//Remote Function Identifiers
#define DOOR_UNLOCK 0x01
#define DOOR_LOCK 0x02
#define REMOTE_START 0x10

//LEDs
#define LED_RESET 0x0000
#define LED_BLUE 0x8000	  //PD15
#define LED_RED 0x4000	  //PD14
#define LED_ORANGE 0x2000 //PD13
#define LED_GREEN 0x1000  //PD12
#define LED_ALL 0xf000

//DELAYs
#define LEDDELAY 200
#define BUTTONDELAY 800
#define WATCHDOGFEEDTIME 10000

// Flash Unlock Sequence
#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

// Variable that will be hold in the flash
#define VARADDR 0x0800C000 // address that will hold the variable
#define VAR 0x12345670	   // variable value

/*************************************************
* Variable Definitions
*************************************************/
static volatile uint32_t tDelay;
static volatile uint32_t timeCounter;
static volatile uint32_t isButtonDown;
static volatile uint32_t buttonPressCounter;
static volatile uint32_t isLongPress;
static volatile uint32_t isBusy;
static volatile uint8_t functionId;
static volatile uint8_t functionId_rx;
static volatile uint8_t err_Id;

static uint8_t dataFrame_tx[16];
static uint8_t key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
static uint8_t dataFrame_rx[16];
const static uint8_t dfConst[16] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a};

static uint8_t receiveDone;
static uint32_t feedtime = 0;
static uint8_t rollingCode_Init = 0x16;

volatile uint8_t tx_complete = 0;
volatile uint8_t rx_complete = 0;
volatile uint8_t bufpos_tx = 0;
volatile uint8_t bufpos_rx = 0;

volatile uint32_t rollingCode_tx; //read from flash
volatile uint32_t rollingCode_rx; //read from flash
volatile uint32_t rollingCounter_tx;
volatile uint32_t rollingCounter_rx;

/*************************************************
* Function Declarations
*************************************************/
void init_systick(uint32_t s, uint8_t cen);
void ErrorHandler(uint8_t errorId);
void WatchdogFeeder();
void ReceiveHandler();
void LED_Init();
void SleepMode();

/*************************************************
* initialize SysTick
*************************************************/
void init_systick(uint32_t s, uint8_t cen)
{
	// Clear CTRL register
	SysTick->CTRL = 0x00000;
	// Main clock source is running with HSI by default which is at 8 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2)
	// 0: Processor clock/8 (AHB/8)
	// 1: Processor clock (AHB)
	SysTick->CTRL |= (0 << 2);
	// Enable callback (bit 1)
	SysTick->CTRL |= ((uint32_t)cen << 1);
	// Load the value
	SysTick->LOAD = s;
	// Set the current value to 0
	SysTick->VAL = 0;
	// Enable SysTick (bit 0)
	SysTick->CTRL |= (1 << 0);
}

/*************************************************
* SysTick Handler
*************************************************/
void SysTick_Handler(void)
{
	if (isButtonDown == 1 && timeCounter < LONG_PRESS_TIME)
	{
		timeCounter++;
	}
	if (tDelay != 0x00)
	{
		tDelay--;
	}
	feedtime++;
	if (feedtime >= WATCHDOGFEEDTIME)
	{
		SCB->SCR = (0 << 1);
	}
}

/*************************************************
* Delay Handler
*************************************************/
void delay_ms(volatile uint32_t s)
{
	tDelay = s;
	while (tDelay != 0)
		;
}

/*************************************************
* Timer 2 Interrupt Handler
*************************************************/
void TIM2_IRQHandler(void)
{
	static uint32_t i = 1;

	// clear interrupt status
	if (TIM2->DIER & 0x01)
	{
		if (TIM2->SR & 0x01)
		{
			TIM2->SR &= ~(1U << 0);
		}
	}

	GPIOD->ODR = (i << 12);

	if (i == 0x08)
	{
		i = 1;
	}
	else
	{
		i = (i << 1);
	}
}

/*************************************************
* Independent Watchdog Interrupt Handler
*************************************************/
void IWDG_IRQHandler(void)
{
	GPIOD->ODR = LED_RED;
}

/*************************************************
* Watchdog Feeder
*************************************************/
void WatchdogFeeder()
{
	IWDG->KR = 0xAAAA;
	SCB->SCR = (1 << 1);
}

/*************************************************
* Button Interrupt Handler
*************************************************/
void EXTI0_IRQHandler(void)
{
	SCB->SCR = (0 << 1);
	isBusy = 0;

	// Check if the interrupt came from exti0
	if (EXTI->PR & (1 << 0))
	{

		//GPIOD->ODR = (uint16_t)(i << 12); //LEDs

		if ((GPIOA->IDR & (1 << 0)) == RISING_EDGE)
		{
			isButtonDown = 1;
			/* do something */
		}

		if ((GPIOA->IDR & (1 << 0)) == FALLING_EDGE)
		{
			if (isLongPress == 1)
			{
				isLongPress = 0;
				isButtonDown = 0;
				timeCounter = 0;
				SCB->SCR = (1 << 1);
			}
			else
			{
				buttonPressCounter++;
				isButtonDown = 0;
				timeCounter = 0;
				/* do something else */
			}
		}

		// Clear pending bit
		EXTI->PR = (1 << 0);
	}
}

/*************************************************
* Setup Button
*************************************************/
void Button_Init(void)
{
	/* set up button */

	GPIOA->MODER &= 0xFFFFFFFC; // Reset bits 0-1 to clear old values
	GPIOA->MODER |= 0x00000000; // Make button an input

	// enable SYSCFG clock (APB2ENR: bit 14)
	RCC->APB2ENR |= (1 << 14);

	/* tie push button at PA0 to EXTI0 */
	// Writing a 0b0000 to pin0 location ties PA0 to EXT0
	SYSCFG->EXTICR[0] |= 0x00000000; // Write 0000 to map PA0 to EXTI0

	// Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
	EXTI->RTSR |= 0x00001; // Enable rising edge trigger on EXTI0
	EXTI->FTSR |= 0x00001; // Enable falling edge trigger on EXTI0

	// Mask the used external interrupt numbers.
	EXTI->IMR |= 0x00001; // Mask EXTI0

	// Set Priority for each interrupt request
	NVIC->IP[EXTI0_IRQn] = 0x10; // Priority level 1

	// enable EXT0 IRQ from NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);
}

/*************************************************
* USART2 Interrupt Handler
*************************************************/
void USART2_IRQHandler(void)
{
	// check if the source is transmit interrupt
	if (USART2->SR & (1 << 7))
	{
		// clear interrupt
		USART2->SR &= (uint32_t) ~(1 << 7);

		if (bufpos_tx == 16)
		{
			// buffer is flushed out, disable tx interrupt
			tx_complete = 1;
			USART2->CR1 &= (uint32_t) ~(1 << 7);
		}
		else
		{
			// flush the next char in the buffer
			tx_complete = 0;
			USART2->DR = dataFrame_tx[bufpos_tx++];
		}
	}
}

/*************************************************
* USART2 Configurator
*************************************************/
void USART2_Init()
{
	// enable USART2 clock, bit 17 on APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	// set pin modes as alternate mode 7 (pins 2 and 3)
	GPIOA->MODER &= 0xFFFFFF0F; // Reset bits 10-15 to clear old values
	GPIOA->MODER |= 0x000000A0; // Set pin 2/3 to alternate func. mode (0b10)

	// set pin modes as high speed
	GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

	// choose AF7 for USART2 in Alternate Function registers
	GPIOA->AFR[0] |= (0x7 << 8); // for pin 2

	// usart2 tx enable, TE bit 3
	USART2->CR1 |= (1 << 3);

	// enable usart2 - UE, bit 13
	USART2->CR1 |= (1 << 13);

	// 12-bit mantissa and 4-bit fraction
	USART2->BRR |= (22 << USART_BRR_DIV_Mantissa_Pos);
	USART2->BRR |= (13 << USART_BRR_DIV_Fraction_Pos);

	NVIC_EnableIRQ(USART2_IRQn);
}

/*************************************************
* USART3 Interrupt Handler
*************************************************/
void USART3_IRQHandler(void)
{
	receiveDone = 0;
	// check if the source is transmit interrupt
	if (USART3->SR & (1 << 5))
	{
		dataFrame_rx[bufpos_rx] = (USART3->DR);
		bufpos_rx++;
		if (bufpos_rx == 16)
		{
			// buffer is read, disable rx interrupt
			rx_complete = 1;
			receiveDone = 1;
			ReceiveHandler();
		}
	}
}

/*************************************************
* USART3 Configurator
*************************************************/
void USART3_Init()
{
	// enable USART3 clock, bit 18 on APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// set pin modes as alternate mode 7 (pins 2 and 3)
	GPIOC->MODER &= ~GPIO_MODER_MODE11_Msk; // Reset bits 10-15 to clear old values
	GPIOC->MODER |= GPIO_MODER_MODE11_1;	// Set pin 10/11 to alternate func. mode (0b10)

	// set pin modes as high speed
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED11_Msk;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED11_1;

	// choose AF7 for USART3 in Alternate Function registers
	GPIOC->AFR[1] |= (AF7 << GPIO_AFRH_AFSEL11_Pos); // for pin 11

	// usart3 rx enable, RE bit 2
	USART3->CR1 |= USART_CR1_RE;

	// enable rx interrupt
	USART3->CR1 |= USART_CR1_RXNEIE;

	// enable usart3 - UE, bit 13
	USART3->CR1 |= USART_CR1_UE;

	// 12-bit mantissa and 4-bit fraction
	USART3->BRR |= (22 << USART_BRR_DIV_Mantissa_Pos);
	USART3->BRR |= (13 << USART_BRR_DIV_Fraction_Pos);

	NVIC_EnableIRQ(USART3_IRQn);
}

/*************************************************
* Flash Handlers
*************************************************/
void unlock_flash()
{
	if (FLASH->CR & FLASH_CR_LOCK)
	{
		FLASH->KEYR = KEY1;
		FLASH->KEYR = KEY2;
	}
}

void lock_flash()
{
	FLASH->CR |= FLASH_CR_LOCK; // bit 31
}

void erase_flash_sector2()
{
	const uint32_t sec = 2;
	__disable_irq();
	while (FLASH->SR & FLASH_SR_BSY)
		; // check if busy
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= (sec << 3);	// SNB bit 3:6
	FLASH->CR |= FLASH_CR_STRT; // start
	while (FLASH->SR & FLASH_SR_BSY)
		;						// check if busy
	FLASH->CR &= ~FLASH_CR_SER; // clear SER bit
	FLASH->CR &= ~(0xFU << 3);	// clear SNB bit 3:6
	__enable_irq();
}

void erase_flash_sector3()
{
	const uint32_t sec = 3;
	__disable_irq();
	while (FLASH->SR & FLASH_SR_BSY)
		; // check if busy
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= (sec << 3);	// SNB bit 3:6
	FLASH->CR |= FLASH_CR_STRT; // start
	while (FLASH->SR & FLASH_SR_BSY)
		;						// check if busy
	FLASH->CR &= ~FLASH_CR_SER; // clear SER bit
	FLASH->CR &= ~(0xFU << 3);	// clear SNB bit 3:6
	__enable_irq();
}

void write_flash(uint32_t addr, uint32_t data)
{
	while (FLASH->SR & FLASH_SR_BSY)
		; // check if busy
	FLASH->CR |= FLASH_CR_PG;
	FLASH->CR &= ~(0x3U << 8); // clear PSIZE bit 8:9
	FLASH->CR |= (0x2 << 8);   // program PSIZE
	*(volatile uint32_t *)addr = data;
	while (FLASH->SR & FLASH_SR_BSY)
		;						 // check if busy
	FLASH->CR &= (~FLASH_CR_PG); // disable PG bit
	FLASH->CR &= ~(0x3U << 8);	 // clear PSIZE bit 8:9
}

/*************************************************
* GPIO Clock Init
*************************************************/
void GPIOClock_Init()
{
	/* Enable GPIOA, GPIOC and GPIOD clock (AHB1ENR: bits 0 and 3) */
	/* AHB1ENR: XXXX XXXX XXXX XXXX XXXX XXXX XXXX 1XXX */
	RCC->AHB1ENR |= 0x0000000D;
}

/*************************************************
* StartUp LEDs Init
*************************************************/
void LED_Init()
{
	GPIOD->MODER &= 0x00FFFFFF; // Reset bits 25:24 to clear old values
	GPIOD->MODER |= 0x55000000; // Set MODER bits 25:24 to 01

	//Blink All LEDs for 1 time 1 second interval
	GPIOD->ODR |= LED_ALL;
	delay_ms(LEDDELAY);
	GPIOD->ODR &= LED_RESET; // Turn off LEDs
	delay_ms(LEDDELAY);
}

/*************************************************
* Error Handler
*************************************************/
void ErrorHandler(uint8_t errorId)
{
	uint8_t err = 0;
	for (err = 0; err <= errorId; err++)
	{
		GPIOD->ODR &= LED_RESET;
		GPIOD->ODR |= LED_RED;
		delay_ms(LEDDELAY);
	}
	err_Id = 0;
	SleepMode();
}

/*************************************************
* Rolling Code Generator
*************************************************/
void RollingCodeGenerator()
{
	if (rollingCode_tx == 0x00)
	{
		rollingCode_tx = (rollingCode_Init + 1) % 256;
	}
	else
	{
		rollingCode_tx = (rollingCode_tx + 1) % 256;
	}
	unlock_flash();
	erase_flash_sector2();
	write_flash(0x08008000, rollingCode_tx);
	lock_flash();
}

/*************************************************
* Rolling Code Generator_rx
*************************************************/
void RollingCodeGenerator_rx()
{
	if (rollingCode_tx == 0x00)
	{
		rollingCode_tx = (rollingCode_Init + 1) % 256;
	}
	else
	{
		rollingCode_rx = (rollingCode_rx + 1) % 256;
	}
	unlock_flash();
	erase_flash_sector3();
	write_flash(0x0800C000, rollingCode_rx);
	lock_flash();
}

/*************************************************
* Frame Encryptor
*************************************************/
void FrameEncryptor()
{
	strcpy(dataFrame_tx, dfConst);
	dataFrame_tx[0] = functionId;
	dataFrame_tx[1] = SRC_ADDRESS;
	dataFrame_tx[2] = DEST_ADDRESS;
	dataFrame_tx[3] = rollingCode_tx;

	struct AES_ctx ctx;

	AES_init_ctx(&ctx, key);
	AES_ECB_encrypt(&ctx, dataFrame_tx);
}

/*************************************************
* Frame Decryptor
*************************************************/
int FrameDecryptor()
{
	RollingCodeGenerator_rx();
	struct AES_ctx ctx;

	AES_init_ctx(&ctx, key);
	AES_ECB_decrypt(&ctx, dataFrame_rx);

	if (dataFrame_rx[1] == SRC_ADDRESS)
	{
		if (dataFrame_rx[2] == DEST_ADDRESS)
		{
			if (dataFrame_rx[3] == rollingCode_rx)
			{
				functionId_rx = dataFrame_rx[0];
				return SUCCESS;
			}
			else
			{
				err_Id = 4; // rolling code not equal
				ErrorHandler(err_Id);
				return FAIL;
			}
		}
		else
		{
			err_Id = 3; // destination address not equal
			ErrorHandler(err_Id);
			return FAIL;
		}
	}
	else
	{
		err_Id = 2; // source address not equal
		ErrorHandler(err_Id);
		return FAIL;
	}
}

/*************************************************
* Light LEDs after Receiving
*************************************************/
void LED_Lighting()
{
	bufpos_rx = 0;
	if (functionId_rx == DOOR_UNLOCK)
	{
		GPIOD->ODR &= LED_RESET;
		GPIOD->ODR |= LED_GREEN;
		SleepMode();
	}
	else if (functionId_rx == DOOR_LOCK)
	{
		GPIOD->ODR &= LED_RESET;
		GPIOD->ODR |= LED_ORANGE;
		SleepMode();
	}
	else if (functionId_rx == REMOTE_START)
	{
		GPIOD->ODR &= LED_RESET;
		GPIOD->ODR |= LED_BLUE;
		SleepMode();
	}
	else
	{
		err_Id = 5;
		ErrorHandler(err_Id);
	}
}

/*************************************************
* Button Press Handler
*************************************************/
void CheckButtonPressType()
{
	bufpos_tx = 0;
	if (buttonPressCounter == SINGLE_PRESS)
	{
		delay_ms(BUTTONDELAY);
		if (buttonPressCounter == DOUBLE_PRESS)
		{
			functionId = DOOR_LOCK;

			//Reset Button Stat do not remove
			buttonPressCounter = 0;
			timeCounter = 0;
			isBusy = 1;

			RollingCodeGenerator();
			FrameEncryptor();
			USART2->CR1 |= (1 << 7);
		}
		else
		{
			functionId = DOOR_UNLOCK;

			//Reset Button Stat do not remove
			buttonPressCounter = 0;
			timeCounter = 0;
			isBusy = 1;

			RollingCodeGenerator();
			FrameEncryptor();
			USART2->CR1 |= (1 << 7);
		}
	}
	if (isButtonDown == 1 && timeCounter >= LONG_PRESS_TIME)
	{
		functionId = REMOTE_START;

		//Reset Button Stat do not remove
		buttonPressCounter = 0;
		timeCounter = 0;
		isLongPress = 1;
		isBusy = 1;

		RollingCodeGenerator();
		FrameEncryptor();
		USART2->CR1 |= (1 << 7);
	}
}

/*************************************************
* ReceiveHandler
*************************************************/
void ReceiveHandler()
{

	if (receiveDone)
	{
		int check;
		check = FrameDecryptor();
		if (check == SUCCESS)
		{
			LED_Lighting();
		}
	}
}

/*************************************************
* Watchdog Initializer
*************************************************/
void Watchdog_Init()
{
	IWDG->KR = 0x5555;
	IWDG->RLR = 4095;
	IWDG->PR = 0x06;
	IWDG->KR = 0xCCCC;
}

/*************************************************
* Sleep Initializer
*************************************************/
void Sleep_Init()
{
	SCB->SCR = (1 << 1);
	__WFI();
}

/*************************************************
* Sleep
*************************************************/
void SleepMode()
{
	isBusy = 0;
	SCB->SCR = (1 << 1);
}

/*************************************************
* Rolling Code Reader TX
*************************************************/
void RollingCodeReader_tx()
{
	//Read Transmitter Rolling Code Data from Flash
	rollingCode_tx = *(uint32_t *)(S2_ADDRESS);
}

/*************************************************
* Rolling Code Reader RX
*************************************************/
void RollingCodeReader_rx()
{
	//Read Receiver Rolling Code Data from Flash
	rollingCode_rx = *(uint32_t *)(S3_ADDRESS);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	/* set system clock to 168 Mhz */
	set_sysclk_to_168();

	init_systick(21000, 1);

	GPIOClock_Init();

	Button_Init();

	USART2_Init();

	USART3_Init();

	RollingCodeReader_tx();

	RollingCodeReader_rx();

	LED_Init();

	Watchdog_Init();

	Sleep_Init();

	while (1)
	{
		if (isBusy != 1)
		{
			CheckButtonPressType();
			WatchdogFeeder();
		}
	}

	if (isBusy == 2)
	{
		return 0;
	}
}
