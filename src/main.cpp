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
#include "protocol_mgr.hpp"
#include "util.hpp"
#include "data.hpp"
#include "minmea.h"

#define STACKSIZE 512

#define ADXL_ADDR 0x53
#define LSM6_ADDR 0x6A
#define LIS3_ADDR 0x1E
#define MS56_ADDR 0x77

#ifndef ARCH_STACK_PTR_ALIGN
#define ARCH_STACK_PTR_ALIGN 8
#endif

static const struct device *const gpib		= DEVICE_DT_GET(DT_NODELABEL(gpiob));
static const struct device *const gpic		= DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *const gpid		= DEVICE_DT_GET(DT_NODELABEL(gpiod));
static const struct device *const adcc1		= DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *const pwm_bz	= DEVICE_DT_GET(DT_NODELABEL(pwm16));
static const struct device *const spi_sen	= DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct device *const i2c_sen	= DEVICE_DT_GET(DT_NODELABEL(i2c2));
static const struct device *const uart_com	= DEVICE_DT_GET(DT_NODELABEL(usart3));
static const struct device *const uart_gps	= DEVICE_DT_GET(DT_NODELABEL(usart1));

static struct adc_channel_cfg adc1c5c = ADC_CHANNEL_CFG_DT(DT_CHILD(DT_NODELABEL(adc1), channel_5));

// Sensors are ring-buffered, except GPS and MS5611 since they works in round robin
RING_BUF_DECLARE(adxl_ring_buf, sizeof(ADXLRawData) * 2);
RING_BUF_DECLARE(lsm6_ring_buf, sizeof(LSM6RawData) * 2);
RING_BUF_DECLARE(lis3_ring_buf, sizeof(LIS3RawData) * 2);
RING_BUF_DECLARE(mti3_ring_buf, sizeof(MTI3RawData) * 2);
MS56RawData ms56_buf;
NMEARawData nmea_buf;

struct spi_config spi_sen_cfg = {
	.frequency = 1000000U,
	.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
};

uint32_t i2c_sen_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;

uint16_t adcbuf;
struct adc_sequence adc_sen_seq = {
	.channels = 0b0000000000100000,
	.buffer = &adcbuf,
	.buffer_size = sizeof(adcbuf),
	.resolution = 12,
};

MTI3 *mti3 = new MTI3(spi_sen, spi_sen_cfg);
ADXL *adxl = new ADXL(i2c_sen, ADXL_ADDR);
LSM6 *lsm6 = new LSM6(i2c_sen, LSM6_ADDR);
LIS3 *lis3 = new LIS3(i2c_sen, LIS3_ADDR);
MS56 *ms56 = new MS56(i2c_sen, MS56_ADDR);

DATCOM *datcom = new DATCOM(uart_com);

// Should use semaphore instead of flags, but whatever
uint8_t sen_i2c_flag = 0;
uint8_t sen_spi_flag = 0;
uint8_t com_send_imu_flag = 0;
uint8_t com_send_gps_flag = 0;
uint8_t ctx_finish_flag = 0;
uint8_t buz_ping_flag = 0;
uint8_t uart_com_flag = 0;
uint8_t uart_gps_flag = 0;

uint8_t pktcounter = 0;
uint8_t rate_buz = 1;
uint8_t uart_com_buf[32];
uint8_t uart_gps_buf[128];

uart_event_rx com_uart_evt;
uart_event_rx gps_uart_evt;

///////////////////////////////////////////////////////////////////////////////
//	PREDECLARATIONS
///////////////////////////////////////////////////////////////////////////////

void sen_poll_exp(struct k_timer *timer);
void gps_poll_exp(struct k_timer *timer);
void dat_ping_exp(struct k_timer *timer);

void buzzer_ping(int16_t freq)
{
	int8_t ret = 0;
	if (freq > 0)
	{
		ret = pwm_set_cycles(pwm_bz, 1, 100000 / freq, 50000 / freq, PWM_POLARITY_NORMAL);
	}
	else
	{
		ret = pwm_set_cycles(pwm_bz, 1, 10, 0, PWM_POLARITY_NORMAL);
	}
}

///////////////////////////////////////////////////////////////////////////////
//	SERIAL INTERRUPT CALLBACK
///////////////////////////////////////////////////////////////////////////////

void com_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_TX_DONE:
		ctx_finish_flag = 1;
		break;
	case UART_TX_ABORTED:
		// should do something here really...
		break;
	case UART_RX_RDY:
		com_uart_evt = evt->data.rx;
		uart_com_flag = 1;
		break;
	default:
		break;
	}
}

void gps_uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type)
	{
	case UART_RX_RDY:
		gps_uart_evt = evt->data.rx;
		uart_gps_flag = 1;
		break;
	default:
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////
//	DATA ACQUISITION
///////////////////////////////////////////////////////////////////////////////

void adxl_get_data()
{
	uint8_t cnt = 2;
	uint8_t adxldata[sizeof(ADXLRawData) * cnt];
	adxl->getMeasurement(adxldata, &cnt);
	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&adxl_ring_buf, &adxldata[i * sizeof(ADXLRawData)], sizeof(ADXLRawData));
	}
}

void lsm6_get_data()
{
	uint8_t cnt = 1;
	uint8_t lsm6data[sizeof(LSM6RawData)];
	lsm6->getMeasurement(lsm6data, &cnt);
	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&lsm6_ring_buf, &lsm6data[i * sizeof(LSM6RawData)], sizeof(LSM6RawData));
	}
}

void lis3_get_data()
{
	uint8_t cnt = 2;
	uint8_t lis3data[6];
	lis3->getMeasurement(lis3data, &cnt);

	for (int8_t i = 0; i < cnt; i++)
	{
		ring_buf_put(&lis3_ring_buf, &lis3data[i * sizeof(LIS3RawData)], sizeof(LIS3RawData));
	}
}

void ms56_get_data()
{
	uint8_t cnt = 1;
	ms56->getMeasurement((uint8_t *)&ms56_buf, &cnt);
}

uint32_t mti3_parse_32b(uint8_t *data)
{
	return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
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
			data->dqA = mti3_parse_32b(&measData[idx + 3]);
			data->dqB = mti3_parse_32b(&measData[idx + 7]);
			data->dqC = mti3_parse_32b(&measData[idx + 11]);
			data->dqD = mti3_parse_32b(&measData[idx + 15]);
			break;
		case XDI_SAMPLETIMEFINE:
			data->sTime = mti3_parse_32b(&measData[idx + 3]);
			break;
		case XDI_PACKETCOUNTER:
			data->counter = mti3_parse_32b(&measData[idx + 3]);
			break;
		case XDI_TEMPERATURE:
			ms56_buf.temp2 = ((mti3_parse_32b(&measData[idx + 3]) >> 8) * 100) >> 12;
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
		mti3->getMeasurement(measData, measCnt);
		MTI3RawData data;
		mti3_parse_measdata(&measData[4], measCnt, &data);
		ring_buf_put(&mti3_ring_buf, (uint8_t *)&data, sizeof(MTI3RawData));
	}

	mti3->getStatus(&notifCnt, &measCnt);
	if (measCnt)
	{
		uint8_t measData[measCnt + 4];
		mti3->getMeasurement(measData, measCnt);
		MTI3RawData data;
		mti3_parse_measdata(&measData[4], measCnt, &data);
		ring_buf_put(&mti3_ring_buf, (uint8_t *)&data, sizeof(MTI3RawData));
	}
	if (notifCnt)
	{
		uint8_t notifData[notifCnt + 4];
		mti3->getNotification(notifData, notifCnt);
	}
}

int8_t mti3_get_msg()
{
	uint16_t notifCnt;
	uint16_t measCnt;

	mti3->getStatus(&notifCnt, &measCnt);
	if (notifCnt)
	{
		uint8_t notifData[notifCnt + 4];
		mti3->getNotification(notifData, notifCnt);
	}
	return notifCnt;
}

void mti3_setup()
{
	uint8_t goMeas[] = {XSENS_GOTOMEASUREMENT, 0};
	mti3->setConfig(goMeas, 2);
}

///////////////////////////////////////////////////////////////////////////////
//	DATCOM PACKET HANDLER
///////////////////////////////////////////////////////////////////////////////

void com_send_imu()
{
	ADXLRawData adxldata = {0};
	LSM6RawData lsm6data = {0};
	LIS3RawData lis3data = {0};
	MTI3RawData mti3data = {0};
	ring_buf_get(&adxl_ring_buf, (uint8_t *)&adxldata, sizeof(ADXLRawData));
	ring_buf_get(&lsm6_ring_buf, (uint8_t *)&lsm6data, sizeof(LSM6RawData));
	ring_buf_get(&lis3_ring_buf, (uint8_t *)&lis3data, sizeof(LIS3RawData));
	ring_buf_get(&mti3_ring_buf, (uint8_t *)&mti3data, sizeof(MTI3RawData));

	uint8_t *data = new uint8_t[52]();

	PROTOCOL_IMU pld_imu{data, 1, pktcounter++};
	pld_imu.putADXLData(&adxldata);
	pld_imu.putLSM6Data(&lsm6data);
	pld_imu.putLIS3Data(&lis3data);
	pld_imu.putMTI3Data(&mti3data);
	pld_imu.finish();

	datcom_payload *pp = new datcom_payload{
		.len = 52,
		.pld = data,
	};

	datcom->putPayload(pp);
}

void com_send_gps()
{
	uint8_t *data = new uint8_t[24]();

	PROTOCOL_GPS pld_gps{data, 2, pktcounter++};
	pld_gps.putNMEAData(&nmea_buf);
	pld_gps.putMS56Data(&ms56_buf);
	pld_gps.finish();

	datcom_payload *pp = new datcom_payload{
		.len = 24,
		.pld = data,
	};

	datcom->putPayload(pp);
}

///////////////////////////////////////////////////////////////////////////////
//	EVENT GENERATORS
///////////////////////////////////////////////////////////////////////////////

void sen_poll_exp(struct k_timer *timer)
{
	sen_i2c_flag = 1;
	sen_spi_flag = 1;
	com_send_imu_flag = 1;
}

void gps_poll_exp(struct k_timer *timer)
{
	com_send_gps_flag = 1;
}

void dat_ping_exp(struct k_timer *timer)
{
	buz_ping_flag = 1;
}

///////////////////////////////////////////////////////////////////////////////
//	THREAD LOOPS
///////////////////////////////////////////////////////////////////////////////

void loop_sen_i2c()
{
	while (1)
	{
		if (sen_i2c_flag)
		{
			sen_i2c_flag = 0;
			adxl_get_data();
			lsm6_get_data();
			lis3_get_data();
			ms56_get_data();
		}

		k_usleep(10);
	}
}

void loop_sen_spi()
{
	while (1)
	{
		if (sen_spi_flag)
		{
			sen_spi_flag = 0;
			mti3_get_data();
		}

		k_usleep(10);
	}
}

void loop_gps()
{
	while (1)
	{
		if (uart_gps_flag)
		{
			uart_gps_flag = 0;
			uart_rx_disable(uart_gps);
			switch (minmea_sentence_id((char *)gps_uart_evt.buf, false))
			{
			case MINMEA_SENTENCE_GGA:
				struct minmea_sentence_gga f_gga;
				if (minmea_parse_gga(&f_gga, (char *)gps_uart_evt.buf))
				{
					nmea_buf.lat = (f_gga.latitude.value / (f_gga.latitude.scale * 100)) * 6000000 + (f_gga.latitude.value % (f_gga.latitude.scale * 100) * (100000 / f_gga.latitude.scale));
					nmea_buf.lon = (f_gga.longitude.value / (f_gga.longitude.scale * 100)) * 6000000 + (f_gga.longitude.value % (f_gga.longitude.scale * 100) * (100000 / f_gga.latitude.scale));
					nmea_buf.alt = f_gga.altitude.value / f_gga.altitude.scale;
					nmea_buf.tim = f_gga.time.seconds + f_gga.time.minutes * 60 + f_gga.time.hours * 3600;
				}
				break;
			case MINMEA_SENTENCE_VTG:
				struct minmea_sentence_vtg f_vtg;
				if (minmea_parse_vtg(&f_vtg, (char *)gps_uart_evt.buf))
				{
					nmea_buf.vel = f_vtg.speed_kph.value * 2 / f_vtg.speed_kph.scale;
					nmea_buf.hdg = f_vtg.true_track_degrees.value * 64 / f_vtg.true_track_degrees.scale;
				}
				break;
			default:
				break;
			}
			memset(uart_gps_buf, 0, sizeof(uart_gps_buf));
			uart_rx_enable(uart_gps, uart_gps_buf, 128, 1000);
		}

		k_usleep(10);
	}
}

void loop_com_imu()
{
	while (1)
	{
		if (uart_com_flag)
		{
			uart_com_flag = 0;
			uart_rx_disable(uart_com);

			if (strcmp((char *)com_uart_evt.buf, "SHUTDOWN") == 0)
			{
				k_timer_stop(&timer_buz);
				k_timer_stop(&timer_sen);
				k_timer_stop(&timer_gps);

				buzzer_ping(2500);
				k_sleep(K_MSEC(200));
				buzzer_ping(1500);
				k_sleep(K_MSEC(200));
				buzzer_ping(750);

				gpio_pin_configure(gpib, 14, GPIO_OUTPUT_INACTIVE);
			}

			memset(uart_com_buf, 0, sizeof(uart_com_buf));
			uart_rx_enable(uart_com, uart_com_buf, 32, 1000);
		}

		if (com_send_imu_flag)
		{
			com_send_imu_flag = 0;
			com_send_imu();
		}

		if (ctx_finish_flag)
		{
			ctx_finish_flag = 0;
			datcom->txFinish();
		}

		k_usleep(10);
	}
}

void loop_com_gps()
{
	while (1)
	{
		if (com_send_gps_flag)
		{
			com_send_gps_flag = 0;
			com_send_gps();
		}

		k_usleep(10);
	}
}

void loop_buz()
{
	while (1)
	{
		if (buz_ping_flag)
		{

			buz_ping_flag = 0;
			buzzer_ping(7500);
			k_msleep(100);
			buzzer_ping(0);
			adc_read(adcc1, &adc_sen_seq);

			if (adcbuf < 2400)
			{
				rate_buz = 1;
			}
			else if (rate_buz != 10)
			{
				rate_buz = 10;
			}

			k_timer_start(&timer_buz, K_SECONDS(rate_buz), K_SECONDS(rate_buz));
		}
		else
		{
			k_usleep(10);
		}
	}
}

K_TIMER_DEFINE(timer_sen, sen_poll_exp, NULL);
K_TIMER_DEFINE(timer_gps, gps_poll_exp, NULL);
K_TIMER_DEFINE(timer_buz, dat_ping_exp, NULL);

K_THREAD_DEFINE(thread_sen_spi,	STACKSIZE, loop_sen_spi,	NULL, NULL, NULL, 1,	0, 0);
K_THREAD_DEFINE(thread_sen_i2c,	STACKSIZE, loop_sen_i2c,	NULL, NULL, NULL, 2,	0, 0);
K_THREAD_DEFINE(thread_gps,		STACKSIZE, loop_gps,		NULL, NULL, NULL, 3,	0, 0);
K_THREAD_DEFINE(thread_com_imu,	STACKSIZE, loop_com_imu,	NULL, NULL, NULL, 4,	0, 0);
K_THREAD_DEFINE(thread_com_gps,	STACKSIZE, loop_com_gps,	NULL, NULL, NULL, 5,	0, 0);
K_THREAD_DEFINE(thread_buz,		STACKSIZE, loop_buz,		NULL, NULL, NULL, 99,	0, 0);

int main()
{
	gpio_pin_configure(gpic, 6, GPIO_OUTPUT_INACTIVE); // LED
	gpio_pin_configure(gpic, 7, GPIO_OUTPUT_INACTIVE); // LED
	gpio_pin_configure(gpid, 2, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpib, 14, GPIO_INPUT);
	gpio_pin_configure(gpid, 3, GPIO_OUTPUT_INACTIVE);

	i2c_configure(i2c_sen, i2c_sen_cfg);

	uart_callback_set(uart_com, com_uart_cb, NULL);
	uart_callback_set(uart_gps, gps_uart_cb, NULL);

	adc_channel_setup(adcc1, &adc1c5c);

	mti3_setup();
	adxl->setup();
	lsm6->setup();
	lis3->setup();
	ms56->setup();

	uart_rx_enable(uart_com, uart_com_buf, 32, 1000);
	uart_rx_enable(uart_gps, uart_gps_buf, 128, 1000);

	buzzer_ping(1000);
	k_sleep(K_MSEC(200));
	buzzer_ping(2000);
	k_sleep(K_MSEC(200));
	buzzer_ping(4000);
	k_sleep(K_MSEC(200));
	buzzer_ping(0);

	k_timer_start(&timer_sen, K_MSEC(500), K_MSEC(10));					 // 100Hz = 10mS
	k_timer_start(&timer_gps, K_MSEC(503), K_MSEC(100));				 // 20Hz = 50mS
	k_timer_start(&timer_buz, K_SECONDS(rate_buz), K_SECONDS(rate_buz)); // 20Hz = 50mS

	uint8_t maincnt = 0;
	while (1)
	{
		if (maincnt++ % 8 == 0)
		{
			gpio_pin_set(gpic, 6, 1);
		}
		else
		{
			gpio_pin_set(gpic, 6, 0);
		}

		k_sleep(K_MSEC(100));
	}
}
