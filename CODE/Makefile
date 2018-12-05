PROGRAM=main
#unsual 19/09/2018 -- #PROGRAM_SRC_FILES = myincludes/hx711.c
PROGRAM_SRC_DIR=. ./myincludes 
EXTRA_COMPONENTS = extras/onewire extras/i2c extras/ds18b20 extras/ds1307 extras/pwm extras/dhcpserver extras/wificfg extras/paho_mqtt_c 
EXTRA_CFLAGS=-DLWIP_MDNS_RESPONDER=1 -DLWIP_NUM_NETIF_CLIENT_DATA=1 -DLWIP_NETIF_EXT_STATUS_CALLBACK=1
include ~/esp8266/esp-open-rtos/common.mk