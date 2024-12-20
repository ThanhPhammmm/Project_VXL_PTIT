#include "config.h"

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
		TIM3->ARR = 50 - 1;   // ~50 Hz multiplexing frequency
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