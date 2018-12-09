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

this project is a prototype, then we use PVC commercial tubes of the 100mm to make mechanical part. however, for helicoidal transport of feed, we maked (via freecad) a design of the helicoid and built it via 3D printer. this part design you can see in folder `MECHANICAL_DESIGN`
    
we inspired from [Helidoid Model](https://www.thingiverse.com/thing:27854)

# Electronic Project

Let's go to the part that interest us. For the control system, we use a microcontroller ESP8266 because it contain various periferics integrated, example is Wi-Fi Module.
    
![PCB 2D](https://github.com/bydansouza/ElectronicPetFeeder/blob/master/PCB_2D.PNG)

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
