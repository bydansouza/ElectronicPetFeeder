/*
 * hx711.h
 *
 *  Created on: 10 de set de 2018
 *      Author: dansouza
 */

#ifndef MYINCLUDES_HX711_H_
#define MYINCLUDES_HX711_H_

#include "espressif/esp_common.h"
#include "esp8266.h"

//#ifdef	__cplusplus
//extern "C" {
//#endif

typedef enum {
	CHANNEL_A,
	CHANNEL_B,
} hx711_channel_t;

typedef enum {
	G128,
	G64,
	G32,
}hx711_channel_gain_t;

typedef struct {
	uint8_t pin_DOUT;
	uint8_t pin_PD_SCK;
	hx711_channel_gain_t A_GAIN;
	hx711_channel_t	CHANNEL;
	unsigned long result;
} hx711_instance_t;

hx711_instance_t* new_hx711(uint8_t pin_DOUT, uint8_t pin_PD_SCK, hx711_channel_gain_t A_GAIN, hx711_channel_t CHANNEL);
void prepare_channel(hx711_instance_t* instance);
void read_channel_a(hx711_instance_t* instance);
void read_channel_b(hx711_instance_t* instance);
uint32_t readhx711(hx711_instance_t* instance);
uint32_t calibrate(hx711_instance_t* instance);
uint32_t convert_dose(uint32_t data); //Convert to mg (miligrams)
uint32_t convert_racao(uint32_t data); //Convert to mg (miligrams)

//#ifdef	__cplusplus
//}
//#endif

#endif /* MYINCLUDES_HX711_H_ */
