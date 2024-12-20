#include "stm32f10x.h"                  // Device header
//#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
//#include "stm32f10x_tim.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#define UART_BUFFER_SIZE 4    // Maximum number of digits you expect

volatile uint32_t number = 0; // Number to display
volatile uint8_t digitIndex = 0;         // Current digit for multiplexing
volatile uint8_t uart_data_ready = 0; // Flag to indicate new UART data
char uart_buffer[4]; // Buffer to store received UART data
uint8_t uart_index = 0;
volatile float timeAcceleration = 1.0f; // Time acceleration factor (1x)
uint8_t digits[4];                     // To hold current digits for display
//volatile uint32_t last_received_time = 0;  // Last time data was received
//volatile uint32_t current_time = 0;  // Current time for timeout
uint32_t over_flag = 0; // determine if there are any received data

const uint8_t Segments[] = {
    0xC0, // 0
    0xF9, // 1
    0xA4, // 2
    0xB0, // 3
    0x99, // 4
    0x92, // 5
    0x82, // 6
    0xF8, // 7
    0x80, // 8
    0x90  // 9
};

void RCC_Config(void);
void GPIO_Config(void);
void USART1_Config(void);
void TIM2_Config(void);
void TIM3_Config(void);
void SysTickConfig(uint32_t ticks);
void CalculateDigits(uint32_t number);
void ProcessButtons(void);

// USART1 Interrupt Handler
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        char received_char = USART1->DR; // Read received character
				over_flag = 0;
        // If the received character is a valid number ('0' to '9')
        if (received_char >= '0' && received_char <= '9') {
            if (uart_index < UART_BUFFER_SIZE) {  // Check buffer overflow
                uart_buffer[uart_index++] = received_char;
            }
        }
    }
}
void CalculateDigits(uint32_t number) {
		digits[0] = number % 10;
		digits[1] = (number / 10) % 10;
		digits[2] = (number / 100) % 10;
		digits[3] = (number / 1000) % 10;
}
//Countdown
void TIM2_IRQHandler(void) {
		if (TIM2->SR &&	 TIM_SR_UIF){
				TIM2->SR &= ~TIM_SR_UIF; // Clear interrupt flag

				if (number > 0){
						number--; // Decrement value
				} 
				else{
						number = 9999; // Reset to 9999
				}
				CalculateDigits(number); // Update digits
		}
}
// Display number on 4x7-segment LEDs
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt flag

        GPIOB->ODR &= ~0xF00; // Disable all digits (set LOW for common anode)
        GPIOA->ODR = Segments[digits[digitIndex]]; // Set segments
        GPIOB->ODR |= (1 << (8 + digitIndex)); // Enable current digit (set HIGH)
        digitIndex = (digitIndex + 1) % 4; // Move to next digit
    }
}
volatile uint32_t buttonPressTime = 0;  // Store last button press time
volatile uint32_t sysTickCounter = 0;  // Global counter incremented in SysTick

void SysTick_Handler(void) {
    sysTickCounter++;  // Increment global counter every millisecond
}

uint32_t GetSysTick(void) {
    return sysTickCounter;  // Return the current counter value
}

void ProcessButtons(void) {
    uint32_t currentTime = GetSysTick();

    if (!(GPIOB->IDR & (1 << 0))) { // PB0 pressed (Increase acceleration)
        if (currentTime - buttonPressTime >= 100) {  // Debounce for 100 ms
            //buttonPressTime = currentTime;  // Update last button press time
            timeAcceleration += 2.0f;  // Increase by 2
            TIM2->PSC = (SystemCoreClock / (10000 * timeAcceleration)) - 1;
				sysTickCounter = 0;
				currentTime = 0;
        }
    }
    if (!(GPIOB->IDR & (1 << 1))) { // PB1 pressed (Decrease acceleration)
        if (currentTime - buttonPressTime >= 100) {  // Debounce for 100 ms
            //buttonPressTime = currentTime;  // Update last button press time
            timeAcceleration -= 2.0f;  // Decrease by 2
            if (timeAcceleration < 0.5f) timeAcceleration = 0.05f;  // Minimum limit
            TIM2->PSC = (SystemCoreClock / (10000 * timeAcceleration)) - 1;
						sysTickCounter = 0;
						currentTime = 0;
        }
    }
		if(!(GPIOB->IDR & (1 << 4))){
			if (currentTime - buttonPressTime >= 100){
				number = 10000;
				//buttonPressTime = currentTime;  // Update last button press time
				sysTickCounter = 0;
				currentTime = 0;
			}
		}
		if (currentTime - buttonPressTime >= 100) {  // Debounce for 100 ms
				//buttonPressTime = currentTime;  // Update last button press time
				timeAcceleration = 1.0f;  // Reset to default value
				TIM2->PSC = (SystemCoreClock / (10000 * timeAcceleration)) - 1;
				sysTickCounter = 0;
				currentTime = 0;
		}
}
/********************************************************************************************************/
int main(void) {
		RCC_Config();
		GPIO_Config();
		USART1_Config();
		TIM2_Config();
		TIM3_Config();
		SysTickConfig(SystemCoreClock / 1000);
		CalculateDigits(number);

		while (1) {
			uint32_t length = strlen(uart_buffer);
			int i;
			over_flag++;
			if((over_flag == 1000) & (length > 0)){
				uart_data_ready = 1;  // Data is ready for processing
				uart_index = 0;
			}
			if(over_flag > 10000){
				over_flag = 1001;
			}
			//USART1_SendString("Hello, USART1!\r\n");
			ProcessButtons(); // Handle button presses
			if (uart_data_ready) {
					number = 0; // Reset the number
					for (i = 0; i < length; i++) {
							if (uart_buffer[i] >= '0' & uart_buffer[i] <= '9') {
									number = number * 10 + (uart_buffer[i] - '0');
							}
					}
					uart_data_ready = 0;  // Reset the flag after processing
					uart_buffer[0] = 0;
					uart_buffer[1] = 0;
					uart_buffer[2] = 0;
					uart_buffer[3] = 0;

					if (number > 9999) number = 9999; // Limit to 4 digits
					CalculateDigits(number);
			}
	}
}
/********************************************************************************************************/
void RCC_Config(void) {
		// Step 1: Enable HSE (High-Speed External oscillator)
		RCC->CR |= RCC_CR_HSEON; // Turn on HSE
		while (!(RCC->CR & RCC_CR_HSERDY)); // Wait for HSE to be ready

		// Step 2: Configure Flash memory latency
		FLASH->ACR |= FLASH_ACR_LATENCY_2; // Set 2 wait states for 72 MHz

		// Step 3: Configure the PLL
		RCC->CFGR &= ~RCC_CFGR_PLLSRC; // Clear PLL source selection
		RCC->CFGR |= RCC_CFGR_PLLSRC; // Set HSE as PLL source
		RCC->CFGR &= ~RCC_CFGR_PLLMULL; // Clear PLL multiplier bits
		RCC->CFGR |= RCC_CFGR_PLLMULL9; // Set PLL multiplier to 9 (8 MHz * 9 = 72 MHz)

		// Step 4: Enable the PLL
		RCC->CR |= RCC_CR_PLLON; // Turn on PLL
		while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to be ready

		// Step 5: Configure the prescalers
		RCC->CFGR &= ~RCC_CFGR_HPRE; // AHB prescaler = 1 (no division)
		RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 prescaler = 1 (no division)
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler = 2 (max 36 MHz)
		RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler = 1 (no division)

		// Step 6: Set PLL as the system clock
		RCC->CFGR &= ~RCC_CFGR_SW; // Clear system clock selection bits
		RCC->CFGR |= RCC_CFGR_SW_PLL; // Select PLL as system clock
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for PLL to be system clock
		
		// Step 7: Enable peripheral clocks
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock (for segments and UART)
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Enable GPIOB clock (for digits and buttons)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1 clock
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable TIM2 clock (countdown timer)
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Enable TIM3 clock (multiplexing)
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;    // Enable AFIO clock (optional)
}
void GPIO_Config(void) {
		// Configure GPIOA (PA0-PA7) as output for LED segments
		GPIOA->CRL = 0x33333333;  // General-purpose output, push-pull
		GPIOA->ODR = 0xFF;        // All segments off

		// Configure GPIOB (PB8, PB9, PB10, PB11) for digit selection
		GPIOB->CRH = 0x00003333;  // PB8, PB9, PB10, PB11 as output
		GPIOB->ODR = 0xF00;       // Disable all digits (PB8-PB11 high)

		// Configure GPIOB (PB0, PB1, PB4) as input for buttons
    GPIOB->CRL = 0x00080088;  // PB0, PB1, PB4 as input with pull-up
    GPIOB->ODR |= (1 << 0) | (1 << 1) | (1 << 3);  // Enable pull-up on PB0, PB1, PB3

		AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;
}
void USART1_Config(void) {
		USART1->BRR = 0x1D4C;          // 9600 baud rate (assuming 72MHz clock)

		// Enable USART receiver and RXNE interrupt
		USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;    // Enable RX and TX
		USART1->CR1 |= USART_CR1_RXNEIE;              // Enable RXNE interrupt
		USART1->CR1 |= USART_CR1_UE;                  // Enable USART
	
		// Configure PA9 (TX)  puas alternate functionsh-pull (USART1_TX)
		//GPIOA->CRH &= ~GPIO_CRH_MODE9;       // Clear MODE bits for PA9
		//GPIOA->CRH |= GPIO_CRH_MODE9_1;      // Set PA9 speed to 2 MHz
		//GPIOA->CRH &= ~GPIO_CRH_CNF9;        // Clear CNF bits for PA9
		//GPIOA->CRH |= GPIO_CRH_CNF9_1;       // Set PA9 to alternate function push-pull
		// Enable USART1 interrupt in NVIC
		NVIC_EnableIRQ(USART1_IRQn);
}
void TIM2_Config(void){
		TIM2->PSC = 7200 - 1;  // Prescaler for 10kHz
		TIM2->ARR = 10000 - 1; // Set auto-reload value for 1-second interrupt
		TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
		TIM2->CR1 |= TIM_CR1_CEN;   // Start TIM2
		NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt in NVIC
}
void TIM3_Config(void){
		TIM3->PSC = 7200 - 1;  // Prescaler for 10kHz
		TIM3->ARR = 50 - 1;   // ~50 Hz multiplexing frequency // 0.005s
		TIM3->DIER |= TIM_DIER_UIE; // Enable update interrupt
		TIM3->CR1 |= TIM_CR1_CEN;   // Start TIM3
		NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt
}
void SysTickConfig(uint32_t ticks) {
    SysTick->LOAD = ticks - 1;           // Set reload register
    SysTick->VAL = 0;                    // Reset the SysTick counter value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | 
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;  // Enable SysTick interrupt and counter
    NVIC_SetPriority(SysTick_IRQn, 0);   // Set the SysTick interrupt priority
}
