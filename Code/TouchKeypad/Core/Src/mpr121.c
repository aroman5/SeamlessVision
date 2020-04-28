
/* ---------------- Inclusions ----------------- */
#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

#include "mpr121.h"
#include "mpr121_def.h"




/* ------------- Local defines --------------- */

/* Uncomment this define to enable MPR121 IRQ interrupt */
//#define MPR121_INT_ENABLED


/* Number of electrodes of MPR121 */
#define NUM_OF_ELECTRODES	13


/* MPR121 I2C bus address */
#define MPR121_ADDRESS_BYTE	0x5A


#ifdef MPR121_INT_ENABLED
/* MPR121 IRQ port, pins, exti defines */
#define MPR121_IRQ_RCC		RCC_GPIOA
#define MPR121_IRQ_PORT		GPIOA
#define MPR121_IRQ_PIN 		GPIO4
#define MPR121_IRQ_EXTI 	EXTI4
#define MPR121_IRQ_isr 		exti4_isr
#define MPR121_IRQ_NVIC 	NVIC_EXTI4_IRQ
#endif




/* ------------- Local functions prototypes ------------ */

static void i2c_init(void);
static void pins_init(void);
static HAL_StatusTypeDef write_register(I2C_HandleTypeDef*, uint8_t, uint8_t);
static HAL_StatusTypeDef read_register(I2C_HandleTypeDef*, uint8_t, uint8_t *);

/* ---------------- Exported functions ----------------- */

/* get touch status */
uint16_t mpr121_get_touch( I2C_HandleTypeDef* hi2c )
{
	uint8_t reg_value = 0;
	uint16_t touch_flags = 0; 

	/* read Touch 1 Status register */
	read_register(hi2c, TS1, &reg_value);
	/* store lower 8 electrodes status flags */
	touch_flags = reg_value;
	/* read Touch 2 Status register */
	read_register(hi2c, TS2, &reg_value);
	/* clear unused higher flags */
	reg_value &= 0x1F;
	/* store higher 5 electrodes status flags */
	touch_flags |= (reg_value << 8);

	return touch_flags;
}


/* MPR121 init */
bool mpr121_init(I2C_HandleTypeDef* hi2c)
{
	bool success;
	uint8_t electrodes_count;
  	uint8_t reg_value = 0;

	success = true;
	HAL_StatusTypeDef status = HAL_OK;

	/* init port pins */
	pins_init();
	/* init I2C interface */
	i2c_init();

	/* soft reset */
	status = write_register(hi2c, SRST, 0x63);

	if(status == HAL_OK)
	{

	}else
	{

	}

	/* read AFE Configuration 2 */
	status = read_register(hi2c, AFE2, &reg_value);

	if(status == HAL_OK)
	{
		success = true;
	}else
	{
		success = false;
	}

	/* check default value */
	if (reg_value != 0x24) {
		/* error */

		success = false;
	} else {
		success = true;
		/* OK */
	}

	/* read Touch Status register */
	status = read_register(hi2c, TS2, &reg_value);
	/* check default value */
	if ((reg_value & 0x80) != 0) {
		/* error */
		success = false;
	} else {
		/* OK */
	}

	/* if no previous error */
	if (success == true)
	{
		/* turn off all electrodes to stop */
		status = write_register(hi2c, ECR, 0x00);

		/* write register with initial values */
		status = write_register(hi2c, MHDR, 0x01);
		status = write_register(hi2c, NHDR, 0x01);
		status = write_register(hi2c, NCLR, 0x10);
		status = write_register(hi2c, FDLR, 0x20);
		status = write_register(hi2c, MHDF, 0x01);
		status = write_register(hi2c, NHDF, 0x01);
		status = write_register(hi2c, NCLF, 0x10);
		status = write_register(hi2c, FDLF, 0x20);
		status = write_register(hi2c, NHDT, 0x01);
		status = write_register(hi2c, NCLT, 0x10);
		status = write_register(hi2c, FDLT, 0xFF);
		status = write_register(hi2c, MHDPROXR, 0x0F);
		status = write_register(hi2c, NHDPROXR, 0x0F);
		status = write_register(hi2c, NCLPROXR, 0x00);
		status = write_register(hi2c, FDLPROXR, 0x00);
		status = write_register(hi2c, MHDPROXF, 0x01);
		status = write_register(hi2c, NHDPROXF, 0x01);
		status = write_register(hi2c, NCLPROXF, 0xFF);
		status = write_register(hi2c, FDLPROXF, 0xFF);
		status = write_register(hi2c, NHDPROXT, 0x00);
		status = write_register(hi2c, NCLPROXT, 0x00);
		status = write_register(hi2c, FDLPROXT, 0x00);
		status = write_register(hi2c, DTR, 0x11);
		status = write_register(hi2c, AFE1, 0xFF);
		status = write_register(hi2c, AFE2, 0x30);
		status = write_register(hi2c, ACCR0, 0x00);
		status = write_register(hi2c, ACCR1, 0x00);
		status = write_register(hi2c, USL, 0x00);
		status = write_register(hi2c, LSL, 0x00);
		status = write_register(hi2c, TL, 0x00);
		status = write_register(hi2c, ECR, 0xCC); // default to fast baseline startup and 12 electrodes enabled, no prox

		if(status == HAL_OK)
		{

		}else
		{

		}

		/* apply next setting for all electrodes */
		for (electrodes_count = 0; electrodes_count < NUM_OF_ELECTRODES; electrodes_count++) {
			status = write_register(hi2c, (E0TTH + (electrodes_count<<1)), 40);
			status = write_register(hi2c, (E0RTH + (electrodes_count<<1)), 20);
		}

		/* enable electrodes and set the current to 16uA */
		status = write_register(hi2c, ECR, 0x10);
	}
	else
	{
		/* init error */
	}

	return success;
}




/* ---------------- Local functions ----------------- */

/* function to write a value to a register into the MPR121. Returned value is not used at the moment */
static HAL_StatusTypeDef write_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t data[2];

	data[0] = reg_addr;
	data[1] = value;

	while (HAL_I2C_Master_Transmit(hi2c, (uint16_t) MPR121_ADDRESS_BYTE, data, 2, 100) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			status = HAL_ERROR;
		}
	}

	return status;
}

/* function to read a register value from the MPR121. Returned value is not used at the moment */
static HAL_StatusTypeDef read_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *receive_buffer)
{
	HAL_StatusTypeDef status = HAL_OK;

	while (HAL_I2C_Master_Transmit(hi2c, (uint16_t) MPR121_ADDRESS_BYTE, &reg_addr, 1, 100) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			status = HAL_ERROR;
		}
	}

	/* Receieve data in the register */
	while (HAL_I2C_Master_Receive(hi2c, MPR121_ADDRESS_BYTE, receive_buffer, 1, 100) != HAL_OK) {

		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
			status = HAL_ERROR;
		}
	}

	return status;
}


/* I2C interface function */
static void i2c_init(void)
{
//	/* Enable I2C1 clock. */
//	rcc_periph_clock_enable(RCC_I2C1);
//	/* Enable I2C1 interrupt. */
//	nvic_enable_irq(NVIC_I2C1_EV_IRQ);
//	/* reset I2C1 */
//	i2c_reset(I2C1);
//	/* standard mode */
//	i2c_set_standard_mode(I2C1);
//	/* clock and bus frequencies */
//	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_2MHZ);
//	i2c_set_ccr(I2C1, 20);
//	/* enable error event interrupt only */
//	i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN);
//	/* enable I2C */
//	i2c_peripheral_enable(I2C1);
}


/* port pins init function */
static void pins_init(void)
{

//#ifdef MPR121_INT_ENABLED
//	/* --- IRQ pin init --- */
//	/* Enable IRQ PORT clock. */
//	rcc_periph_clock_enable(MPR121_IRQ_RCC);
//
//	/* Enable MPR121 IRQ interrupt. */
//	nvic_enable_irq(MPR121_IRQ_NVIC);
//
//	/* set MPR121 IRQ as input with external pull-up resistors */
//	gpio_mode_setup(MPR121_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, MPR121_IRQ_PIN);
//
//	/* Configure the EXTI subsystem. */
//	exti_select_source(MPR121_IRQ_EXTI, MPR121_IRQ_PORT);
//	exti_set_trigger(MPR121_IRQ_EXTI, EXTI_TRIGGER_FALLING);
//
//	/* enable MPR121 interrupt */
//	exti_enable_request(MPR121_IRQ_EXTI);
//#endif
//
//	/* --- I2C interface init --- */
//	/* Enable GPIOB clock. */
//	rcc_periph_clock_enable(RCC_GPIOB);
//
//	/* set I2C1_SCL and I2C1_SDA, external pull-up resistors */
//	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
//	/* Open Drain, Speed 100 MHz */
//	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7);
//	/* Alternate Function: I2C1 */
//	gpio_set_af(GPIOB, GPIO_AF4,  GPIO6 | GPIO7);

}


#ifdef MPR121_INT_ENABLED
/* MPR121 IRQ function */
void MPR121_IRQ_isr(void)
{
	exti_reset_request(MPR121_IRQ_EXTI);

	/* do what you want... */
}
#endif



/* End of file */

