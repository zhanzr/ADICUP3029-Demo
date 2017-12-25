#include <stdint.h>
#include <common.h>

#include <adi_processor.h>
#include <drivers/pwr/adi_pwr.h>
#include <drivers/gpio/adi_gpio.h>

typedef struct {
	ADI_GPIO_PORT Port;
	ADI_GPIO_DATA Pins;
} PinMap;

/* LED GPIO assignments */
PinMap LDS4 = {ADI_GPIO_PORT1, ADI_GPIO_PIN_15};  /*   Blue LED on GPIO31 (DS4) */
PinMap LDS3 = {ADI_GPIO_PORT2, ADI_GPIO_PIN_0};   /* Green LED on GPIO32 (DS3) */

extern uint32_t SystemCoreClock;

int main(void)
{
	uint8_t         gpioMemory[ADI_GPIO_MEMORY_SIZE] = {0};
	ADI_PWR_RESULT  ePwrResult;
	ADI_GPIO_RESULT eGpioResult;

	ePwrResult = adi_pwr_Init();

	ePwrResult = adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1u);

	ePwrResult = adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1u);
	
	/* common init */
	common_Init();
	
	/* Initialize GPIO driver */
	eGpioResult= adi_gpio_Init(gpioMemory, ADI_GPIO_MEMORY_SIZE);

	/* Enable LDS4 output */
	eGpioResult = adi_gpio_OutputEnable(LDS4.Port, LDS4.Pins, true);

	/* Enable LDS3 output */
	eGpioResult = adi_gpio_OutputEnable(LDS3.Port, LDS3.Pins, true);
	
	printf("ADICUP3029 LED UART Demo by zhanzr21 for 21ic BBS @ %u Hz\n", SystemCoreClock);

	/* Loop indefinitely */
	while (1)  
	{
		adi_gpio_SetHigh(LDS3.Port,  LDS3.Pins);
		/* Delay between iterations */
		for (volatile uint32_t i = 0; i < 1000000; i++)
					;

		adi_gpio_SetLow(LDS3.Port,  LDS3.Pins);		
		/* Delay between iterations */
		for (volatile uint32_t i = 0; i < 1000000; i++)
					;		

		adi_gpio_SetHigh (LDS4.Port, LDS4.Pins);
		/* Delay between iterations */
		for (volatile uint32_t i = 0; i < 500000; i++)
					;		

		adi_gpio_SetLow (LDS4.Port, LDS4.Pins);
		/* Delay between iterations */
		for (volatile uint32_t i = 0; i < 500000; i++)
					;			
	}

	return 0;
}
	