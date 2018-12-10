# About this! 
Electronic Pet Feeder Project - Automatic Feeder for Dogs and Cats ;)

By Daniel Henrique Camargo de Souza @ PIN22107 - IFSC/Electronic Engineering/Professors: Daniel Lohmann AND Robinson Pizzio - Dezembro/2018

# Descript:
* This project consists of an electronic automatic feeder for dogs and cats, and this project were made from course Integrator Project class in undergraduate at Electronic Engineering by IFSC/Florianópolis@Brazil.

# Guide to using this repository folders and files
Here you find this structure of folders and files

* `CODE` contains the main code, the Makefile and includes for compile code to microcontroller ESP8266. 

* `ELECTRONIC_DESIGN` contains the files for electronic scheme and PCB design of the circuit for the control system.

* `MECHANICAL_DESIGN` contain the files of mechanical part, essentially the helicoid part design, other parts were made from commercials items.

* `_guides` contain files for learning about sensor and controllers used in this project, essentially datasheets and manuals

* `_models` contain files for other electronic pet feeder, but this was made with Arduino (sic) 

* `_patents` contain files of patents discovered about electronic pet feeders.

* `_refs` Others references files

# Steps for sucess:

* 1) Market research:
    
    ** First we proceded in a market research, seeking out a similar product. **
    
    List of similar products:
    
    [10 Best Automatic Pet Feeders 2017](https://www.youtube.com/watch?v=MUnrIJn3qq8)
    
    [10 Best Automatic Pet Feeders 2018](https://www.youtube.com/watch?v=D8cHpOWlC1c)
    
    [JEMPET Petwant SmartFeeder Automatic Pet Feeder](http://www.petwant.com/)
    
    [Automatic Cat Feeder | PF-10 CAT](https://www.petfeedster.com/product/automatic-cat-feeder/)
        
    ** in following, we search the prices of these similar products **
    
    [JEMPET Petwant SmartFeeder Automatic Pet Feeder](https://www.amazon.com/Petwant-SmartFeeder-Automatic-Dispenser-Controlled/dp/B01GFTZPDQ)
    
    [Smart Feed Automatic Pet Feeder](https://store.petsafe.net/smart-feed)
    
    [Automatic Cat Feeder | PF-10 CAT](https://www.amazon.com/New-Pet-Feedster-PF-10-PLUS/dp/B01N9D4672)
    
    
* 2) Diferenciais Attributes for this project:

    All configs via MQTT
    
    Notification for user 
    
    Water control, Temperature and level

# Mechanical Project

This project is a prototype, then we use PVC commercial tubes of the 100mm to make mechanical part. however, for helicoidal transport of feed, we maked (via freecad) a design of the helicoid and built it via 3D printer. this part design you can see in folder `MECHANICAL_DESIGN`
    
We inspired from [Helidoid Model](https://www.thingiverse.com/thing:27854) to construct the helicoid part. This is inside into PVC tube of the 100mm of format "T". in the base of the helicoid have a motor that spin this helicoid liberating the feed.

The Helicoid design part coudl be seen in the next image. 
![PCB 2D](https://github.com/bydansouza/ElectronicPetFeeder/blob/master/HELICOID.PNG)

The feed fall into the pot and under it (in the base) have a load cell for 2Kg, controlled from MCU (ESP8266). The client set, with MQTT commands, a quantity of feed and the system control the motor to spin the necessary to liberate a quantity seted.

In the base of system, have too another load cell, of the 50Kg, that monitoring the quantity of feed that fall into the pot (exited of system load). inside in code have a routine that calculate the variation of load, that fall into the pot and exited of system load, this two quantities should are equals.

For monitoring and control the water input and output, have been installed two solenoid valves. Inside the water pot have a temperature sensor and level sensor that through MCU controller we monitoring this two variables, and controled the start or stop of solenoids valves (if necessary).

# Electronic Project

Let's go to the part that interest us. For the control system, we used a microcontroller ESP8266 because it contain various periferics integrated, example is Wi-Fi Module.

Together of the MCU microcontroller, we used two accessories periferics. The first is HX711, Load cell controller that communicate with MCU part trough two pins serial interface, we implemented the librarie for this communicate with ESP8266 (see you into `CODE`). The second is RTC (Real Time Controller) based in DS1307, this is used in control of the time, and for this we used the librarie avaiable in `esp-open-rtos`

For the water temperature control, we used the DS18B20 Sensor (because this is water resistant), then for communicate with MCU microcontroller this sensor use one wire communication, and this is implemented in the librarie included in `esp-open-rtos`.

In the water level control, we used a capacitive sensor connected in input of 555 oscillator. according that the level rises, the oscillation produced in 555 change, the capacitive sensor controll the frequency of the oscillation in 555, thus we have a converter of the water level in frequency of oscilation. the second stage is connect the output os the 555 in an VCO (Voltage Control Oscilator) configured to convert frequency oscilations in voltages levels, thus we have the complete cicle, and we connect this output voltage level in an analog to digital converter of the MCU Controller.

For the water temperature control, we used the DS18B20 Sensor (because this is water resistant), then for communicate with MCU microcontroller this sensor use one wire communication, and this is implemented in the librarie included in `esp-open-rtos`.

In the water level control, we used a capacitive sensor connected in input of 555 oscillator. according that the level rises, the oscillation produced in 555 change, the capacitive sensor controll the frequency of the oscillation in 555, thus we have a converter of the water level in frequency of oscilation. the second stage is connect the output os the 555 in an VCO (Voltage Control Oscilator) configured to convert frequency oscilations in voltages levels, thus we have the complete cicle, and we connect this output voltage level in an analog to digital converter of the MCU Controller.

The PCB design could be seen in the next image.  
![PCB 2D](https://github.com/bydansouza/ElectronicPetFeeder/blob/master/PCB_2D.PNG)

In this design we priorise the final dimension of board and the disposition os the components in board for reduce EMI an another other possibles mistakes.

The 3D PCB design could be seen in the next image. 
![PCB 3D](https://github.com/bydansouza/ElectronicPetFeeder/blob/master/PCB_3D.PNG)

## Main Code

* this code is based in esp-open-rtos sdk to ESP8266, then recommended to use the [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos)

* `[Here](https://github.com/xtarke/kairos/tree/master/codes/mqtt)`

# References
[1](https://www.instructables.com/id/Automatic-Arduino-Powered-Pet-Feeder/)
[2](https://www.hackster.io/circuito-io-team/iot-pet-feeder-10a4f3)
[3](https://www.circuito.io/blog/automatic-pet-feeder/)
[4](https://circuitdigest.com/microcontroller-projects/automatic-pet-feeder-using-arduino)
[5](https://www.youtube.com/watch?v=hpQ21NZ_fuw)
