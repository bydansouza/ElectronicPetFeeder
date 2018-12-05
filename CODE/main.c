/*
 * main.c
 *	Main code to pi3 project in Electronic Engineering undergraduate Course at IFSC/Florianópolis-SC@Brasil
 *
 *      Author: Daniel Henrique Camargo de Souza, @byDansouza
 *
 *      Colaboration: xtarke (Prof:. Renan Starke)
 */
// Header files ----------------------------------------------------------------
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>

#include "esp8266.h"
#include "timers.h"
#include <queue.h>
#include <semphr.h>
#include "lwip/api.h"

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>
#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <paho_mqtt_c/MQTTSubscribe.h>

//Drivers Headers Files
#include "myincludes/ds18b20.h"		//Temperature Sensor
#include "myincludes/hx711.h"		//Load Cell amplifier
#include "myincludes/i2c.h"			//i2c Driver to RTC DS1307
#include "myincludes/ds1307.h"		//RTC DS1307 Driver
#include "myincludes/pwm.h"			//PWM Driver

#include "myincludes/network.h"
#include "myincludes/comm.h"
#include <sysparam.h>
#include "myincludes/app_status.h"

//Definitions
#include "myincludes/defPrincipais.h"

#define SENSOR_GPIO 13			//Temperature PIN Sensor
#define MAX_SENSORS 1			//Numbers of Temperatura Sensor in system
#define RESCAN_INTERVAL 1		//Scans in each sensor
#define LOOP_DELAY_MS 5000		//Temperature contain high inertia value, then the delay time can high

#define DOUT 12					//hx711 pin DOUT
#define PD_SCK 14				//hx711 pin PDSCK

#define I2C_BUS 1				//I2C Bus
#define SCL_PIN 5				//SCL I2C PIN
#define SDA_PIN 4				//SDA I2C PIN

#define MOTOR_PIN 0				//Motor PIN
#define WINPUT_PIN 2			//Water Input valve PIN
#define WOUTPUT_PIN 10			//Water Output valve PIN

#define WATER_SENSOR 15			//Capacitive Water Sensor PIN

//Initial Params For tests
#define NMAX_DOSE 6				//Numero máximo de doses diárias
#define DOSE_RACAO 30
#define DOSE_MIN 1
#define DOSE_HOUR 12
#define DELAY_TASKS 60000

static QueueHandle_t tsqueue;
static SemaphoreHandle_t xBinaryControl;

typedef enum petfeecomm_t{
	SET_NDOSE,
	SET_DOSE,
	SET_HOUR,
	SET_MIN,
	SET_WTEMP
}petfeecomm_t;

typedef struct {
	union
	{
		struct
		{
			uint32_t fIsTime		: 1;	//Chegou a Hora da Dose
			uint32_t fEqualDose		: 1;	//Dose Igualou
			uint32_t fMotorDose		: 1;	//Start/Stop do Motor da Dose
			uint32_t fWaterLevel	: 1;	//Chegou no Nível de Agua
			uint32_t fWaterTemp		: 1;	//Temperatura Ultrapassou
			uint32_t fWaterInput	: 1;	//Start/Stop entrada de agua
			uint32_t fWaterOutput	: 1;	//Start/Stop Saída de agua
			uint32_t fUnusedBits	: 25;	//Flags não usadas (fUnusedBits+Others = 32bits)
		};
		uint32_t fAllFlags;					//All Flags
	}			feeder_flags;			//Flags de Controle
	uint8_t 	feeder_nDose;			//Numero de Doses Diárias programadas
	uint32_t 	feeder_min[NMAX_DOSE];	//Minutos das Doses programadas
	uint32_t 	feeder_hour[NMAX_DOSE];	//Horas das Doses programadas
	uint32_t 	feeder_mday;			//Marca o Dia, para não repetir horários
	uint32_t 	feeder_dose;			//Quatidade da Dose Programada
	uint32_t	feeder_delay;			//Delay para as demais Tarefas
	float 		feeder_waterTemp;		//Temperatura da Agua permitida
}petFeederControl_t;
static petFeederControl_t petFeederControl;		//Create Static Global Variable

#define DEBUG

#ifdef DEBUG
#define debug(fmt, ...) printf("%s: " fmt "\n", "DEBUG", ## __VA_ARGS__)
#else
#define debug(fmt, ...)
#endif

static void  status_publish_task(void *pvParameters){
	uint8_t count = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();

    publisher_data_t to_publish;

    char *wifi_my_host_name = NULL;

	/* Get MQTT host IP from web server configuration */
	sysparam_get_string("hostname", &wifi_my_host_name);

	if (!wifi_my_host_name){
		printf("Invalid host name\n");
		strncpy(to_publish.topic,"host_1/heartbeat",PUB_TPC_LEN);
	}else{
		strncpy(to_publish.topic, wifi_my_host_name,PUB_TPC_LEN);
		free(wifi_my_host_name);
		strncat(to_publish.topic,"/heartbeat",PUB_TPC_LEN);
	}

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);

        //temperature = get_temperature();

        snprintf(to_publish.data, PUB_MSG_LEN, "Beat %d\r\n", count++);

        if (xQueueSend(publish_queue, (void *)&to_publish, 0) == pdFALSE) {
        	debug("Publish queue overflow.\r\n");
        }
    }
}

static void  status_subscrib_task(void *pvParameters){
    int size;
    uint8_t incH,i;
	command_data_t rx_data;

    while(1){
        while(xQueueReceive(command_queue, (void *)&rx_data, (TickType_t)10000) == pdTRUE){
            debug("Command: %d\n",rx_data.cmd);
            debug("Data: %lu\n",rx_data.data);

            switch (rx_data.cmd){
				case SET_NDOSE:
					petFeederControl.feeder_nDose = (uint8_t)rx_data.data;
					incH = petFeederControl.feeder_nDose/18;				//18 horas uteis para tratar animais
					break;
				case SET_DOSE:
					petFeederControl.feeder_dose = rx_data.data/petFeederControl.feeder_nDose;		//Dose Diaria / Numero de Doses no dia
					break;
				case SET_HOUR:
					petFeederControl.feeder_hour[0] = rx_data.data;
					for(i=1;i<petFeederControl.feeder_nDose;i++){
						petFeederControl.feeder_hour[i] = petFeederControl.feeder_hour[i-1]+incH;		// Set demais horarios
					}
					break;
				case SET_MIN:
					for(i=0;i<petFeederControl.feeder_nDose;i++){
						petFeederControl.feeder_min[i] = rx_data.data;
					}
					break;
				case SET_WTEMP:
					petFeederControl.feeder_waterTemp = (float)rx_data.data;
					break;
            }
        }
    }
}

void gpio_intr_handler(){
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendFromISR(tsqueue, &now, NULL);
}

void mensure_waterlevel(void *pvParameters){
    gpio_enable(WATER_SENSOR, GPIO_INPUT);
	gpio_set_interrupt(WATER_SENSOR, GPIO_INTTYPE_EDGE_NEG, gpio_intr_handler);
    uint32_t period;
    uint32_t last = 0;

    while(1) {
    	xQueueReceive(tsqueue, &period, (TickType_t)100 );

        if(last < period) {

        	debug("\t\t\tPeriod is %d[ms]\r\n", (period-last)*portTICK_PERIOD_MS);

        	//IMPLEMENTAR O NÍVEL DE AGUA		02/12/2018

            last = period;
        }
    }
}

void realtime_task(void *pvParameters){
	uint8_t i;
	i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_400K);

	i2c_dev_t dev = {
		.addr = DS1307_ADDR,
	    .bus = I2C_BUS,
	};

	// setup datetime: 2016-11-1 12:00:01
	struct tm time = {
		.tm_year = 2018,
	    .tm_mon  = 11,  // 0-based
	    .tm_mday = 1,
	    .tm_hour = 12,
	    .tm_min  = 00,
	    .tm_sec  = 01
	};

	ds1307_start(&dev, TRUE);

	if (ds1307_is_running(&dev)){
		ds1307_set_time(&dev, &time);
	}else{
		ds1307_start(&dev, TRUE);
	}

	while(1){
		ds1307_get_time(&dev, &time);

		debug("\t%04d-%02d-%02d %02d:%02d:%02d\n", time.tm_year, time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

		//Comparar Horario (Minuto e Hora) com setado
		for(i=0;i<petFeederControl.feeder_nDose;i++){
			if (time.tm_min == petFeederControl.feeder_min[i] && time.tm_hour == petFeederControl.feeder_hour[i]){
					petFeederControl.feeder_flags.fIsTime = TRUE;			//Chegou a Hora da Ração
					petFeederControl.feeder_delay = 100;					//Deixa Todas as Tasks em ALERTA
			}else{
				petFeederControl.feeder_flags.fIsTime = FALSE;
			}
		}
		vTaskDelay(DELAY_TASKS/portTICK_PERIOD_MS); //wait 1min
	}
}

void mensure_loadcell(void *pvParameters){
	uint32_t _dose;
	uint32_t _racao;

	hx711_instance_t* _feeder = new_hx711(DOUT,PD_SCK,G64,CHANNEL_A);
	hx711_instance_t* _pot = new_hx711(DOUT,PD_SCK,G32,CHANNEL_B);

	while(1){
		prepare_channel(_pot); 						//Prepare channel and gain -- setting pulse numbers
		_dose = readhx711(_pot)&0xFFFFFB00;  		//Elimina os 10bits menos significativos
		if (_dose < 8427520) _dose = 8427520;		//Valor da Tara --> 8427520 (Medida em 02/11/2018)

		debug("Dose = %lu\n",convert_dose(_dose)/1000);	//Converte para miligramas(mg) e Divide por 1000 para fornecer em gramas (g)

		prepare_channel(_feeder);
		_racao = readhx711(_feeder)&0xFFFFFB00;  		//Elimina os 10bits menos significativos
		if (_racao < 8357888) _racao = 8357888;			//FALTA CALIBRAR A RACAO??? (28/11/2018)

		debug("\t\tRacao = %lu\n",convert_racao(_racao)/1000);

		if (convert_dose(_dose)/1000 > petFeederControl.feeder_dose){
			petFeederControl.feeder_flags.fEqualDose = TRUE;
		}else{
			petFeederControl.feeder_flags.fEqualDose = FALSE;
		}

		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

void mensure_watertemp(void *pvParameters) {
    ds18b20_addr_t addrs[MAX_SENSORS];
    float temps[MAX_SENSORS];
    int sensor_count;

    // There is no special initialization required before using the ds18b20
    // routines.  However, we make sure that the internal pull-up resistor is
    // enabled on the GPIO pin so that one can connect up a sensor without
    // needing an external pull-up (Note: The internal (~47k) pull-ups of the
    // ESP8266 do appear to work, at least for simple setups (one or two sensors
    // connected with short leads), but do not technically meet the pull-up
    // requirements from the DS18B20 datasheet and may not always be reliable.
    // For a real application, a proper 4.7k external pull-up resistor is
    // recommended instead!)

    gpio_set_pullup(SENSOR_GPIO, TRUE, TRUE);

    while(1) {
        // Every RESCAN_INTERVAL samples, check to see if the sensors connected
        // to our bus have changed.
        sensor_count = ds18b20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS);

        if (sensor_count < 1) {
			#ifdef DEBUG
            //printf("\nNo sensors detected!\n");
			#endif
        } else {
            //printf("\n%d sensors detected:\n", sensor_count);
            // If there were more sensors found than we have space to handle,
            // just report the first MAX_SENSORS..
            if (sensor_count > MAX_SENSORS) sensor_count = MAX_SENSORS;

            // Do a number of temperature samples, and print the results.
            for (int i = 0; i < RESCAN_INTERVAL; i++) {
                ds18b20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
                for (int j = 0; j < sensor_count; j++) {
                    // The DS18B20 address is a 64-bit integer, but newlib-nano
                    // printf does not support printing 64-bit values, so we
                    // split it up into two 32-bit integers and print them
                    // back-to-back to make it look like one big hex number.
                    uint32_t addr0 = addrs[j] >> 32;
                    uint32_t addr1 = addrs[j];
                    float temp_c = temps[j];
                    float temp_f = (temp_c * 1.8) + 32;

                    debug("Sensor %08x%08x reports: %f \xC2\xB0""C [%f \xC2\xB0""F] \n", addr0, addr1, temp_c, temp_f);

                    //FALTA IMPLEMENTAR UMA TEMPORIZAÇÃO -- Verificar se a temperatura se mantem assim por certo periodo de tempo (30min por exemplo)
                    // se sim... acionar o mecanismo de trca de agua
                    if(temp_c > petFeederControl.feeder_waterTemp){
                    	petFeederControl.feeder_flags.fWaterTemp = TRUE;		//Temperatura Ultrapassou
                    }else{
                    	petFeederControl.feeder_flags.fWaterTemp = FALSE;
                    }
                }

                // Wait for a little bit between each sample (note that the
                // ds18b20_measure_and_read_multi operation already takes at
                // least 750ms to run, so this is on top of that delay).
                vTaskDelay(LOOP_DELAY_MS / portTICK_PERIOD_MS);
            }
        }
    }
}

void control_motor(void *pvParameters) {
	gpio_enable(MOTOR_PIN, GPIO_OUTPUT);
	gpio_write(MOTOR_PIN,0);

	while(1){
		if(petFeederControl.feeder_flags.fMotorDose == TRUE){
			gpio_write(MOTOR_PIN,1);
			debug("Set MOTOR_PIN\n");

			if (petFeederControl.feeder_flags.fEqualDose == TRUE){
				gpio_write(MOTOR_PIN,0);
				debug("Reset MOTOR_PIN\n");

				petFeederControl.feeder_flags.fMotorDose = FALSE;
				petFeederControl.feeder_delay = DELAY_TASKS;
			}

			vTaskDelay(100/portTICK_PERIOD_MS);
		}else{
			gpio_write(MOTOR_PIN,0);
			vTaskDelay(petFeederControl.feeder_delay/portTICK_PERIOD_MS);
		}
	}
}

void control_waterinput(void *pvParameters) {
	gpio_enable(WINPUT_PIN, GPIO_OUTPUT);
	gpio_write(WINPUT_PIN,0);

	while(1){
		if(petFeederControl.feeder_flags.fWaterInput == TRUE){
			gpio_write(WINPUT_PIN,1);
			debug("Set WINPUT_PIN \n");

			if (petFeederControl.feeder_flags.fWaterLevel == TRUE){		//Nível de Agua = 1 (No Topo)
				gpio_write(WINPUT_PIN,0);
				debug("Reset WINPUT_PIN \n");

				petFeederControl.feeder_flags.fWaterInput = FALSE;
			}
			vTaskDelay(100/portTICK_PERIOD_MS);
		}else{
			gpio_write(WINPUT_PIN,0);
			vTaskDelay(petFeederControl.feeder_delay/portTICK_PERIOD_MS);
		}
	}
}

void control_wateroutput(void *pvParameters) {
	gpio_enable(WOUTPUT_PIN, GPIO_OUTPUT);
	gpio_write(WOUTPUT_PIN,0);

	while(1){
		if(petFeederControl.feeder_flags.fWaterOutput == TRUE){
			gpio_write(WOUTPUT_PIN,1);
			debug("Set WOUTPUT_PIN \n");

			if (petFeederControl.feeder_flags.fWaterLevel == FALSE){		//Nível de Agua = 0 (ZEROU)
				gpio_write(WOUTPUT_PIN,0);
				debug("Reset WOUTPUT_PIN \n");

				petFeederControl.feeder_flags.fWaterOutput = FALSE;
			}
			vTaskDelay(100/portTICK_PERIOD_MS);
		}else{
			gpio_write(WOUTPUT_PIN,0);
			vTaskDelay(petFeederControl.feeder_delay/portTICK_PERIOD_MS);
		}
	}
}

void control_FSM(void *pvParameters) {

	while(1){
		if(petFeederControl.feeder_flags.fIsTime == TRUE){
			petFeederControl.feeder_flags.fMotorDose = TRUE;
			debug("Is Time \n");
		}else{
			debug("No Time \n");

			petFeederControl.feeder_delay = DELAY_TASKS;
		}
		vTaskDelay(DELAY_TASKS/portTICK_PERIOD_MS);
	}
}

void user_init(void) {
    uart_set_baud(0, 115200);

    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    // Set led to indicate wifi status.
    sdk_wifi_status_led_install(2, PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);

    xBinaryControl = xSemaphoreCreateBinary();
    xSemaphoreGive(xBinaryControl);					//Release the Semaphore

    petFeederControl.feeder_flags.fAllFlags = 0;
//    petFeederControl.feeder_dose = DOSE_RACAO;
//    petFeederControl.feeder_nDose = NMAX_DOSE;
//    petFeederControl.feeder_min[0] = DOSE_MIN;
//    petFeederControl.feeder_hour[0] = DOSE_HOUR;
    petFeederControl.feeder_delay = DELAY_TASKS;

    publish_queue = xQueueCreate(8, sizeof(publisher_data_t));
    command_queue = xQueueCreate(4, sizeof(command_data_t));
    tsqueue = xQueueCreate(2, sizeof(uint32_t));

    wifi_cfg();
    //trick by xtarke --> 'make erase_flash' to erase/renew flash memory of ESP8266

    xTaskCreate(&mensure_waterlevel, "WaterLevelMensure", 256, NULL, 2, NULL);
    xTaskCreate(&mensure_watertemp, "WaterTempMensure", 256, NULL, 2, NULL);
    xTaskCreate(&mensure_loadcell, "LoadCellMensure", 256, NULL, 2, NULL);
    xTaskCreate(&realtime_task, "DS1307", 256, NULL, 2, NULL);
    xTaskCreate(&control_motor, "MotorControl", 256, NULL, 2, NULL);
    xTaskCreate(&control_waterinput, "WinputControl", 256, NULL, 2, NULL);
    xTaskCreate(&control_wateroutput, "WoutputControl", 256, NULL, 2, NULL);
    xTaskCreate(&control_FSM, "FiniteStateMachine", 256, NULL, 2, NULL);

    xTaskCreate(&status_publish_task, "PublishComm", 256, NULL, 2, NULL);
    xTaskCreate(&status_subscrib_task, "SubscribComm", 256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 7, NULL);

//Test MQTT
    // SUBSCRIVE to listen publish from PetFeeder Heatbeat
    // mosquitto_sub -v -h 172.16.0.2 -p 1883 -t eor-b27071/heartbeat

    // PUBLISH Command to PetFeeder
    // mosquitto_pub -m "1w0" -t eor-b27071/cfg/0
}
