# About this! 
Electronic Pet Feeder Project - Automatic Feeder for Dogs and Cats ;)

By Daniel Henrique Camargo de Souza @ PIN22107 - IFSC/Electronic Engineering/Professors: Daniel Lohmann AND Robinson Pizzio

Dezembro/2018

# Descript:
* This project consist in a electronic automatic feeder for dogs and cats, and this maked from course Integrated Project class in undergraduate at Electronic Engineering by IFSC/Florianópolis@Brazil.


# Steps for building:

* 1)' Market research:
    --> From Internet, 

* 2) Patent's research:


* 3) Similar Products:


* 4) Diferenciais Attributes:


# Mechanical Project


# Electronic Project


# Main Code

* Use the [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos),

* `[Here](https://github.com/xtarke/kairos/tree/master/codes/mqtt)`


# References
    - https://www.thingiverse.com/thing:27854
    - https://www.instructables.com/id/Automatic-Arduino-Powered-Pet-Feeder/
    - https://www.hackster.io/circuito-io-team/iot-pet-feeder-10a4f3
    - https://www.circuito.io/blog/automatic-pet-feeder/
    - https://circuitdigest.com/microcontroller-projects/automatic-pet-feeder-using-arduino
    - https://www.youtube.com/watch?v=hpQ21NZ_fuw
    
    
# Guide to use this repository folders and files
    
    
    





This Project
- Automatic pet feeder 
    (Diferenciais: 
        - Todas as Configurações via APP
            - Dosagem por Peso {Concorrentes são pelo tempo, não tendo precisão no peso da dosagem}
            - Horarios da Dose
        - Notifição/Monitoramento da quantidade da ração/alimento (Falta ou Acabando/Reposição) [Monitor do Nível]
        - Controle da Agua
            - PH e Temperatura (SOL)
            - Troca da agua automatica (Quando quente ou àcida)
            - Marcação dos horarios em que o PET bebeu agua, com projeção de quantidade em mL
        - Alimentação por bateria
            - Boa Autonomia
            - Recarga via painel solar
        - Camera para monitoramento a distância (Fotos e Vidêos)
        
        
    Pesquisa de Mercado
    
    - Pesquisa nos bancos de patentes
    
    - Levantamento de Concorrentes
    
        - https://www.youtube.com/watch?v=MUnrIJn3qq8
    
        - https://www.youtube.com/watch?v=D8cHpOWlC1c
        
        JEMPET Petwant SmartFeeder Automatic Pet Feeder
        - http://www.petwant.com/
        
        Automatic Cat Feeder | PF-10 CAT
        - https://www.petfeedster.com/product/automatic-cat-feeder/
        
    
    - Levantamento dos preços
    
    JEMPET Petwant SmartFeeder Automatic Pet Feeder
    - https://www.amazon.com/Petwant-SmartFeeder-Automatic-Dispenser-Controlled/dp/B01GFTZPDQ
    - https://www.americanas.com.br/produto/32786320/alimentador-automatico-para-pets-petwant-smart-pet-feeder-pf-103-com-camera-branco-preto?WT.srch=1&epar=bp_pl_00_go_pet_todas_geral_gmv&gclid=CjwKCAjwkrrbBRB9EiwAhlN8_EXzHM2zU3_0T45zzqoFAMBEaPDmdMxqzq4u6ue_NW6odtSgQ0e-RBoCcgkQAvD_BwE&opn=YSMESP&sellerId=7413725000150
    
    Smart Feed Automatic Pet Feeder
    - https://store.petsafe.net/smart-feed
    
    Automatic Cat Feeder | PF-10 CAT
    - https://www.amazon.com/New-Pet-Feedster-PF-10-PLUS/dp/B01N9D4672
    
    
****************************** EXAMPLE *************************
VHDL Project of the MIPS architecture processor
By Daniel HC Souza @ PLD22109 - IFSC/Electronic Engineering/Professor:. Renan Starke
July/2018

_IOs/ --> VHD files for periferics processor unit (IO parts)
	io_buttons.vhd 		--> buttons and leds for DE10-lite
	io_displays.vhd 	--> 7-seg displays for DE10-lite
		io_displays_binarytodisplay.vhd --> Conversor BCD code to 7-seg displays
	io_spi.vhd 		--> VHD to Gsensor periferic of the DE10-lite
	io_uart.vhd 	--> VHD for UART communication in the DE10-lite
		io_uart_fsm.vhd 		--> FSM of the UART
		io_uart_transmit.vhd 	--> Periferic of the transmit UART communication
		
_MPU/ --> VHD files for parts of the processor arch (Registers, ALU, Memory, etc)
	bancoreg.vhd		--> 32 registers bank (MIPS Arch)
		muxreg.vhd		--> 32 vias mux for Register Bank
		regn.vhd		--> Register of the Register Bank
	branch.vhd			--> Branch SUM (ULA), and control branchs in MIPS Arch
	decode.vhd			--> Decode FUNCT 5 bits @ MIPS ISA
	extendsignal.vhd	--> Extend Signal @ MIPS ISA
	fsm.vhd				--> MIPS FSM (Finite State Machine)
	memcontrol.vhd		--> RAM Memory Control, Integrated with periferics
	mux.vhd				--> General Proposit 32bits MUX
	regnIR.vhd			--> Decoder of the Instructions (Instruction Registers)
	regnPC.vhd			--> Program Counter
	rom.vhd				--> Program Memory - ROM
	ula.vhd				--> Unit Logic Aritmetic

mipsbydan.vhd --> VHD design of MIPS version ByDan Processor (Similar of the MIPS arch)
testbench_mipsbydan.vhd --> VHD file for simulation with Modelsim
testbench_mipsbydan.do --> Automation file for simulation with Modelsim
******************** CAUTION ***********************************
For simulation in Modelsim, Insert comment lines 276 to 280 (IP ROM from Quartus), and discomment lines 282 to 291 (Internal Designed ROM). Paste in this Internal ROM the codes insided in 003_CODE/
****************************************************************

###################### OTHER LOCATIONS
001_PROJ_QUARTUS/		--> Sintetize Project Quartus for DE10-lite
002_PROJ_XILINS/		--> Sintetize Project Xilins (no used)
003_CODE/				--> Codes for ROM, writing in MARS - MIPS simulator code
004_FIGS/				--> Interesting figures
005_REFS/				--> Theoritical references
006_DATASHEETS/			--> Factories Manuals
007_OTHERS/				--> Others VHD files, previously utilized 
