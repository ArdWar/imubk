#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/ring_buffer.h>

#include "sen_mti3.hpp"
#include "sen_adxl.hpp"
#include "sen_lsm6.hpp"
#include "sen_lis3.hpp"
#include "sen_ms56.hpp"
#include "com_uart.hpp"
#include "util.hpp"
#include "data.hpp"

#define STACKSIZE 512

#define ADXL_ADDR 0x53
#define LSM6_ADDR 0x6A
#define LIS3_ADDR 0x1E
#define MS56_ADDR 0x77

static const struct device *const gpia = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *const gpic = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *const gpid = DEVICE_DT_GET(DT_NODELABEL(gpiod));
static const struct device *const adcc1 = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *const pwm_bz = DEVICE_DT_GET(DT_NODELABEL(pwm16));
static const struct device *const spi1_d = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct device *const i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
static const struct device *const uart_3 = DEVICE_DT_GET(DT_NODELABEL(usart3));

static struct adc_channel_cfg adc1c5c = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc1), channel_5));

RING_BUF_DECLARE(adxl_ring_buf, sizeof(ADXLRawData) * 2);
RING_BUF_DECLARE(lsm6_ring_buf, sizeof(LSM6RawData) * 2);
RING_BUF_DECLARE(lis3_ring_buf, sizeof(LIS3RawData) * 2);
RING_BUF_DECLARE(mti3_ring_buf, sizeof(MTI3RawData) * 2);
RING_BUF_DECLARE(ms56_ring_buf, sizeof(MS56RawData) * 2);

struct spi_config spi1_cfg = {
	.frequency = 1000000U,
	.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
};

MTI3 *mti3 = new MTI3(spi1_d, spi1_cfg);
ADXL *adxl = new ADXL(i2c_dev, ADXL_ADDR);
LSM6 *lsm6 = new LSM6(i2c_dev, LSM6_ADDR);
LIS3 *lis3 = new LIS3(i2c_dev, LIS3_ADDR);
MS56 *ms56 = new MS56(i2c_dev, MS56_ADDR);

DATCOM *datcom = new DATCOM(uart_3);

uint16_t adcbuf;
struct adc_sequence sequence = {
	.channels = 0b0000000000100000,
	.buffer = &adcbuf,
	.buffer_size = sizeof(adcbuf),
	.resolution = 12,
};

uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
uint8_t sensorpoolflag = 0;
// uint8_t spisensorpoolflag = 0;
uint8_t datcomsendflag = 0;
uint8_t buzzerpingflag = 0;
uint8_t buzzerpingrate = 1;

void serial_cb(const struct device *dev, struct uart_event *evt, void *user_data);

void serial_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_TX_DONE:
		datcom->txFinish();
		break;
	case UART_TX_ABORTED:
		break;
	case UART_RX_RDY:
		break;
	case UART_RX_BUF_REQUEST:
		break;
	case UART_RX_BUF_RELEASED:
		break;
	case UART_RX_DISABLED:
		break;
	case UART_RX_STOPPED:
		break;
	default:
		break;
	}
}

void adxl_get_data()
{
	uint8_t cnt = sizeof(ADXLRawData) * 2;
	uint8_t adxldata[cnt];
	adxl->getMeasurement(adxldata, &cnt);
	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&adxl_ring_buf, &adxldata[i * sizeof(ADXLRawData)], sizeof(ADXLRawData));
	}
}

void lsm6_get_data()
{
	uint8_t cnt = sizeof(LSM6RawData) * 2;
	uint8_t lsm6data[cnt];
	lsm6->getMeasurement(lsm6data, &cnt);
	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&lsm6_ring_buf, &lsm6data[i * sizeof(LSM6RawData)], sizeof(LSM6RawData));
	}
}

void lis3_get_data()
{
	uint8_t cnt = sizeof(LIS3RawData) * 2;
	uint8_t lis3data[cnt];
	lis3->getMeasurement(lis3data, &cnt);
	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&lis3_ring_buf, &lis3data[i * sizeof(LIS3RawData)], sizeof(LIS3RawData));
	}
}

void ms5611_get_data()
{
	uint8_t cnt = sizeof(MS56RawData) * 2;
	uint8_t ms56data[cnt];
	lis3->getMeasurement(ms56data, &cnt);
	ring_buf_put(&lis3_ring_buf, ms56data, sizeof(MS56RawData));
}

uint32_t mti3_parse_32b(uint8_t *data)
{
	return data[0] << 8 | data[1] << 0; // | data[2]<<8 | data[3];
}

int8_t mti3_parse_measdata(uint8_t *measData, uint8_t len, MTI3RawData *data)
{
	if (len < 2)
	{
		return -1;
	}

	if (measData[0] != XSENS_MTDATA2)
	{
		return -2;
	}

	if (len - 3 != measData[1])
	{
		return -3;
	}

	uint8_t idx = 2;

	while (idx < (len - 3))
	{
		uint16_t dataID = ((uint16_t)measData[idx + 1] | ((uint16_t)measData[idx] << 8));
		uint8_t dataLen = measData[idx + 2];
		switch (dataID & 0xFFFF0)
		{
		case XDI_ACCELERATION:
			data->accX = mti3_parse_32b(&measData[idx + 3]);
			data->accY = mti3_parse_32b(&measData[idx + 7]);
			data->accZ = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_RATEOFTURN:
			data->rotX = mti3_parse_32b(&measData[idx + 3]);
			data->rotY = mti3_parse_32b(&measData[idx + 7]);
			data->rotZ = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_MAGNETICFIELD:
			data->magX = mti3_parse_32b(&measData[idx + 3]);
			data->magY = mti3_parse_32b(&measData[idx + 7]);
			data->magZ = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_EULERANGLES:
			data->roll = mti3_parse_32b(&measData[idx + 3]);
			data->pitch = mti3_parse_32b(&measData[idx + 7]);
			data->yaw = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_DELTAV:
			data->dvX = mti3_parse_32b(&measData[idx + 3]);
			data->dvY = mti3_parse_32b(&measData[idx + 7]);
			data->dvZ = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_DELTAQ:
			data->dqX = mti3_parse_32b(&measData[idx + 3]);
			data->dqY = mti3_parse_32b(&measData[idx + 7]);
			data->dqZ = mti3_parse_32b(&measData[idx + 11]);
			break;
		case XDI_SAMPLETIMEFINE:
			data->sTime = mti3_parse_32b(&measData[idx + 3]);
			break;
		case XDI_PACKETCOUNTER:
			data->sTime = mti3_parse_32b(&measData[idx + 3]);
			break;
		default:
			break;
		}
		idx += dataLen + 3;
	}

	return 0;
}

void mti3_get_data()
{
	uint16_t notifCnt;
	uint16_t measCnt;

	mti3->getStatus(&notifCnt, &measCnt);
	if (measCnt)
	{
		uint8_t measData[measCnt + 4];
		int8_t r = mti3->getMeasurement(measData, measCnt);
		MTI3RawData data;
		mti3_parse_measdata(&measData[4], measCnt, &data);
		ring_buf_put(&mti3_ring_buf, (uint8_t *)&data, sizeof(MTI3RawData));
	}

	mti3->getStatus(&notifCnt, &measCnt);
	if (measCnt)
	{
		uint8_t measData[measCnt + 4];
		int8_t r = mti3->getMeasurement(measData, measCnt);
		MTI3RawData data;
		mti3_parse_measdata(&measData[4], measCnt, &data);
		ring_buf_put(&mti3_ring_buf, (uint8_t *)&data, sizeof(MTI3RawData));
	}
	if (notifCnt)
	{
		uint8_t notifData[notifCnt + 4];
		int8_t r = mti3->getNotification(notifData, notifCnt);
	}
}

void mti3_setup()
{
	uint8_t goConfig[] = {XSENS_GOTOCONFIG, 0};
	mti3->setConfig(goConfig, sizeof(goConfig));

	uint16_t setFormat[] = {XSENS_SETOUTPUTCONFIGURATION | 36 << 8,
							XDI_PACKETCOUNTER, 0xFFFF,
							XDI_SAMPLETIMEFINE, 0xFFFF,
							XDI_EULERANGLES | XDI_ENU | XDI_I32, 0xFFFF,
							XDI_ACCELERATION | XDI_I32, 0xFFFF,
							XDI_RATEOFTURN | XDI_I32, 0xFFFF,
							XDI_MAGNETICFIELD | XDI_I32, 0xFFFF,
							XDI_DELTAV | XDI_I32, 0xFFFF,
							XDI_DELTAQ | XDI_I32, 0xFFFF,
							XDI_STATUSWORD, 0xFFFF};
	mti3->swapEndian(setFormat, sizeof(setFormat));
	mti3->setConfig((uint8_t *)setFormat, sizeof(setFormat));

	uint8_t goMeas[] = {XSENS_GOTOMEASUREMENT, 0};
	mti3->setConfig(goMeas, sizeof(goMeas));
}

void uart_dummy()
{
	// for(uint16_t i=0;i<43;i++) {
	// 	data[i] = buzzerpingrate;
	// 	// adxldata[i] = (int)((float)adxldata[i] * 3.14159265359);
	// 	// adxldata[i] /= i;
	// }
	MTI3RawData mtidata;
	ring_buf_get(&mti3_ring_buf, (uint8_t *)&mtidata, sizeof(MTI3RawData));

	// printk("A % 6d % 6d % 6d\n", mtidata.accX, mtidata.accY, mtidata.accZ);

	uint8_t *data = new uint8_t[50]();
	// for(uint16_t i=0;i<50;i++) {
	// 	data[i] = buzzerpingrate*16+i;
	// }
	data[0] = 0xa5;
	data[1] = buzzerpingrate;

	datcom_payload *pp = new datcom_payload{
		.len = 50,
		.pld = data,
	};

	datcom->putPayload(pp);
	// gpio_pin_set(gpid, 2, 1);
	// uart_tx(uart_3, &comdata[0], 43, SYS_FOREVER_US);
}

void sen_poll_exp(struct k_timer *timer)
{
	sensorpoolflag = 1;
	datcomsendflag = 1;
	// spisensorpoolflag = 1;
}

void dat_send_exp(struct k_timer *timer)
{
}

void dat_ping_exp(struct k_timer *timer)
{
	buzzerpingflag = 1;
}

void sensorpool()
{
	while (1)
	{
		if (sensorpoolflag)
		{
			sensorpoolflag = 0;
			mti3_get_data();
			adxl_get_data();
		}
		else
		{
			k_usleep(100);
		}
	}
}

void datcomsend()
{
	while (1)
	{
		if (datcomsendflag)
		{
			datcomsendflag = 0;
			buzzerpingrate++;
			uart_dummy();
			// printk('ping\n');
		}
		else
		{
			k_usleep(100);
		}
	}
}

K_TIMER_DEFINE(sen_poll_timer, sen_poll_exp, NULL);
K_TIMER_DEFINE(dat_send_timer, dat_send_exp, NULL);
K_TIMER_DEFINE(dat_ping_timer, dat_ping_exp, NULL);

void buzzerping()
{
	while (1)
	{
		if (buzzerpingflag)
		{
			buzzerpingflag = 0;
			uint16_t val = adc_read(adcc1, &sequence);
			buzzerpingrate = 0;
			// if(adcbuf < 2400) {
			// 	buzzerpingrate = 1;
			// 	k_timer_start(&dat_ping_timer, K_MSEC(507), K_MSEC((uint16_t)buzzerpingrate*100));
			// } else if (buzzerpingrate != 1) {
			// 	buzzerpingrate = 1;
			// 	k_timer_start(&dat_ping_timer, K_MSEC(507), K_MSEC((uint16_t)buzzerpingrate*100)); //20Hz = 50mS
			// }

			uint8_t *data = new uint8_t[25]();
			for (uint16_t i = 0; i < 25; i++)
			{
				data[i] = i;
			}

			datcom_payload *pp = new datcom_payload{
				.len = 25,
				.pld = data,
			};

			datcom->putPayload(pp);

			// printk("PING!!! ADC:%02X %d\n\n", val, adcbuf);
			pwm_set_cycles(pwm_bz, 1, 20, 0, PWM_POLARITY_NORMAL);
			k_msleep(50);
			pwm_set_cycles(pwm_bz, 1, 20, 0, PWM_POLARITY_NORMAL);
		}
		else
		{
			k_usleep(100);
		}
	}
}

K_THREAD_DEFINE(sensorthread, STACKSIZE, sensorpool, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(datcomthread, STACKSIZE, datcomsend, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(buzzerthread, STACKSIZE, buzzerping, NULL, NULL, NULL, 3, 0, 0);

int main()
{
	printk("\n\n\n\nHello!\n\n");
	i2c_configure(i2c_dev, i2c_cfg);
	gpio_pin_configure(gpic, 6, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpic, 7, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpid, 2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpia, 8, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpid, 3, GPIO_OUTPUT_INACTIVE);
	uart_callback_set(uart_3, serial_cb, NULL);
	adc_channel_setup(adcc1, &adc1c5c);

	mti3_setup();
	adxl->setup();
	lsm6->setup();
	lis3->setup();
	ms56->setup();

	pwm_set_cycles(pwm_bz, 1, 100, 50, PWM_POLARITY_NORMAL);
	k_msleep(200);
	pwm_set_cycles(pwm_bz, 1, 50, 25, PWM_POLARITY_NORMAL);
	k_msleep(200);
	pwm_set_cycles(pwm_bz, 1, 25, 12, PWM_POLARITY_NORMAL);
	k_msleep(200);
	pwm_set_cycles(pwm_bz, 1, 20, 0, PWM_POLARITY_NORMAL);

	k_timer_start(&sen_poll_timer, K_MSEC(500), K_MSEC(10));							 // 100Hz = 10mS
	k_timer_start(&dat_send_timer, K_MSEC(503), K_MSEC(50));							 // 20Hz = 50mS
	k_timer_start(&dat_ping_timer, K_MSEC(507), K_MSEC((uint16_t)buzzerpingrate * 100)); // 20Hz = 50mS

	while (1)
	{
		gpio_pin_toggle(gpic, 6);
		gpio_pin_toggle(gpic, 7);
		// gpio_pin_toggle(gpia, 8);
		k_sleep(K_MSEC(1000));
	}
}
