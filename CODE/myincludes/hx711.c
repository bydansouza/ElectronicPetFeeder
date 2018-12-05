/*
 * hx711.c
 *
 *  Created on: 10 de set de 2018
 *      Author: dansouza
 */

#include "hx711.h"

//#define DEBUG

#ifdef DEBUG
#define debug(fmt, ...) printf("%s: " fmt "\n", "DEBUG", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

hx711_instance_t* new_hx711(uint8_t pin_DOUT, uint8_t pin_PD_SCK, hx711_channel_gain_t A_GAIN, hx711_channel_t CHANNEL){

	hx711_instance_t* new = (hx711_instance_t*)malloc(sizeof(hx711_instance_t));

	if(new == NULL){
		fprintf(stderr, "Not create HX711 Instance!\n");
		exit(EXIT_FAILURE);
	}

	//Initialize
	new->pin_DOUT = pin_DOUT;
	new->pin_PD_SCK = pin_PD_SCK;
	new->A_GAIN = A_GAIN;
	new->CHANNEL = CHANNEL;
	new->result = 0;

	gpio_enable(new->pin_DOUT, GPIO_INPUT);
	gpio_enable(new->pin_PD_SCK, GPIO_OUTPUT);

	return(new);
}

void prepare_channel(hx711_instance_t* instance){
	unsigned char i;

	if(instance->CHANNEL == CHANNEL_A){
		if(instance->A_GAIN == G128){
			while(gpio_read(instance->pin_DOUT));
			for(i=0;i<25;i++){							//25 pulses clock -- next conversion is CH.A, GAIN 128
				sdk_os_delay_us(1);
				gpio_write(instance->pin_PD_SCK,1);
				sdk_os_delay_us(1);
				gpio_write(instance->pin_PD_SCK,0);
			}
		}else if(instance->A_GAIN == G64){
			while(gpio_read(instance->pin_DOUT));
			for(i=0;i<27;i++){							//27 pulses clock -- next conversion is CH.A, GAIN 64
				sdk_os_delay_us(1);
				gpio_write(instance->pin_PD_SCK,1);
				sdk_os_delay_us(1);
				gpio_write(instance->pin_PD_SCK,0);
			}
		}
	}else if(instance->CHANNEL == CHANNEL_B){
		while(gpio_read(instance->pin_DOUT));
		for(i=0;i<26;i++){								//26 pulses clock -- next conversion is CH.B, GAIN 32
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,1);
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,0);
		}
	}
}

void read_channel_a(hx711_instance_t* instance){
	unsigned char i;
	unsigned long result;

	result = 0;
	gpio_write(instance->pin_PD_SCK,0);
	while(gpio_read(instance->pin_DOUT));
	if(instance->A_GAIN == G128){
		for(i=0;i<24;i++){
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,1);
			result = result<<1;
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,0);
			if(gpio_read(instance->pin_DOUT)) result++;
		}
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,1);
		result = result^0x800000;
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,0);
		instance->result = result;
	}else if(instance->A_GAIN == G64){
		for(i=0;i<24;i++){
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,1);
			result = result<<1;
			sdk_os_delay_us(1);
			gpio_write(instance->pin_PD_SCK,0);
			if(gpio_read(instance->pin_DOUT)) result++;
		}
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,1);
		result = result^0x800000;
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,0);
		instance->result = result;
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,1);
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,0);
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,1);
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,0);
	}
}

void read_channel_b(hx711_instance_t* instance){
	unsigned char i;
	unsigned long result;

	result = 0;
	gpio_write(instance->pin_PD_SCK,0);
	while(gpio_read(instance->pin_DOUT));
	for(i=0;i<24;i++){
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,1);
		result = result<<1;
		sdk_os_delay_us(1);
		gpio_write(instance->pin_PD_SCK,0);
		if(gpio_read(instance->pin_DOUT)) result++;
	}
	sdk_os_delay_us(1);
	gpio_write(instance->pin_PD_SCK,1);
	result = result^0x800000;
	sdk_os_delay_us(1);
	gpio_write(instance->pin_PD_SCK,0);
	instance->result = result;
	sdk_os_delay_us(1);
	gpio_write(instance->pin_PD_SCK,1);
	sdk_os_delay_us(1);
	gpio_write(instance->pin_PD_SCK,0);
}

uint32_t readhx711(hx711_instance_t* instance){
	if(instance->CHANNEL == CHANNEL_A){
		read_channel_a(instance);
	}else if(instance->CHANNEL == CHANNEL_B){
		read_channel_b(instance);
	}

	return instance->result;
}

uint32_t calibrate(hx711_instance_t* instance){
	if(instance->CHANNEL == CHANNEL_A){
		read_channel_a(instance);
	}else if(instance->CHANNEL == CHANNEL_B){
		read_channel_b(instance);
	}

	return instance->result;
}

//Convert to mg (miligrams)
uint32_t convert_dose(uint32_t data){
	uint32_t out;

	out = 4.2067*(data)-35452047;	//Linear Equation

	return out;
}

uint32_t convert_racao(uint32_t data){
	uint32_t out;

	out = 4.2067*(data)-35452047;	//Linear Equation

	return out;
}
