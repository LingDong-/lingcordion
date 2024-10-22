EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L fab:R R3
U 1 1 643FE1B6
P 3400 5200
F 0 "R3" V 3193 5200 50  0000 C CNN
F 1 "R20K" V 3284 5200 50  0000 C CNN
F 2 "fab:R_1206" V 3330 5200 50  0001 C CNN
F 3 "~" H 3400 5200 50  0001 C CNN
	1    3400 5200
	0    1    1    0   
$EndComp
$Comp
L fab:C C4
U 1 1 643FE9E0
P 3100 5200
F 0 "C4" V 2848 5200 50  0000 C CNN
F 1 "C0.39uF" V 2939 5200 50  0000 C CNN
F 2 "fab:C_1206" H 3138 5050 50  0001 C CNN
F 3 "" H 3100 5200 50  0001 C CNN
	1    3100 5200
	0    1    1    0   
$EndComp
Text Label 2950 5200 2    50   ~ 0
Audio_Input
$Comp
L fab:R R4
U 1 1 644006E9
P 3900 4750
F 0 "R4" H 3830 4704 50  0000 R CNN
F 1 "R20K" H 3830 4795 50  0000 R CNN
F 2 "fab:R_1206" V 3830 4750 50  0001 C CNN
F 3 "~" H 3900 4750 50  0001 C CNN
	1    3900 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 4900 3900 5200
Connection ~ 3900 5200
Wire Wire Line
	3900 5200 4000 5200
Wire Wire Line
	5000 5200 4800 5200
Text Label 5150 5200 0    50   ~ 0
Vo1
Wire Wire Line
	5150 5200 5000 5200
Connection ~ 5000 5200
Text Label 6850 4600 0    50   ~ 0
SpVo1
Text Label 6850 4700 0    50   ~ 0
SpVo2
Text Label 5000 5650 0    50   ~ 0
Vo2
Wire Wire Line
	5000 5650 4800 5650
$Comp
L power:VDD #PWR018
U 1 1 64403C5A
P 4900 4450
F 0 "#PWR018" H 4900 4300 50  0001 C CNN
F 1 "VDD" H 4915 4623 50  0000 C CNN
F 2 "" H 4900 4450 50  0001 C CNN
F 3 "" H 4900 4450 50  0001 C CNN
	1    4900 4450
	1    0    0    -1  
$EndComp
$Comp
L fab:AMP_MONO_LM4871M U3
U 1 1 643F1598
P 4400 5425
F 0 "U3" H 4400 4858 50  0000 C CNN
F 1 "AMP_MONO_LM4871M" H 4400 4949 50  0000 C CNN
F 2 "fab:SOIC-8_3.9x4.9mm_P1.27mm" H 4400 5425 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 4400 5425 50  0001 C CNN
	1    4400 5425
	1    0    0    1   
$EndComp
Wire Wire Line
	4800 5350 4900 5350
Wire Wire Line
	4900 5350 4900 4600
$Comp
L fab:C C6
U 1 1 644054ED
P 4750 4600
F 0 "C6" V 4498 4600 50  0000 C CNN
F 1 "C1uF" V 4589 4600 50  0000 C CNN
F 2 "fab:C_1206" H 4788 4450 50  0001 C CNN
F 3 "" H 4750 4600 50  0001 C CNN
	1    4750 4600
	0    1    1    0   
$EndComp
Connection ~ 4900 4600
Wire Wire Line
	4900 4600 4900 4450
$Comp
L power:GND #PWR016
U 1 1 64405A87
P 4600 4600
F 0 "#PWR016" H 4600 4350 50  0001 C CNN
F 1 "GND" V 4605 4472 50  0000 R CNN
F 2 "" H 4600 4600 50  0001 C CNN
F 3 "" H 4600 4600 50  0001 C CNN
	1    4600 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR017
U 1 1 64406528
P 4800 5500
F 0 "#PWR017" H 4800 5250 50  0001 C CNN
F 1 "GND" V 4805 5372 50  0000 R CNN
F 2 "" H 4800 5500 50  0001 C CNN
F 3 "" H 4800 5500 50  0001 C CNN
	1    4800 5500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 5350 3850 5350
Wire Wire Line
	3850 5350 3850 5500
Wire Wire Line
	3850 5500 4000 5500
$Comp
L fab:C C5
U 1 1 644074EF
P 3700 5500
F 0 "C5" V 3448 5500 50  0000 C CNN
F 1 "C1uF" V 3539 5500 50  0000 C CNN
F 2 "fab:C_1206" H 3738 5350 50  0001 C CNN
F 3 "" H 3700 5500 50  0001 C CNN
	1    3700 5500
	0    1    1    0   
$EndComp
Connection ~ 3850 5500
$Comp
L power:GND #PWR015
U 1 1 644079C0
P 3550 5500
F 0 "#PWR015" H 3550 5250 50  0001 C CNN
F 1 "GND" V 3555 5372 50  0000 R CNN
F 2 "" H 3550 5500 50  0001 C CNN
F 3 "" H 3550 5500 50  0001 C CNN
	1    3550 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 5650 4000 5650
Wire Wire Line
	3550 5200 3900 5200
$Comp
L Analog_DAC:MCP4901 U4
U 1 1 66C1702A
P 7050 5500
F 0 "U4" H 7794 5546 50  0000 L CNN
F 1 "MCP4901" H 7794 5455 50  0000 L CNN
F 2 "fab:SOIC-8_3.9x4.9mm_P1.27mm" H 8050 5400 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/22248a.pdf" H 8050 5400 50  0001 C CNN
	1    7050 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 66C17C75
P 6650 5400
F 0 "#PWR020" H 6650 5150 50  0001 C CNN
F 1 "GND" V 6655 5272 50  0000 R CNN
F 2 "" H 6650 5400 50  0001 C CNN
F 3 "" H 6650 5400 50  0001 C CNN
	1    6650 5400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR024
U 1 1 66C18218
P 7050 5900
F 0 "#PWR024" H 7050 5650 50  0001 C CNN
F 1 "GND" V 7055 5772 50  0000 R CNN
F 2 "" H 7050 5900 50  0001 C CNN
F 3 "" H 7050 5900 50  0001 C CNN
	1    7050 5900
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR023
U 1 1 66C18AFF
P 7050 5100
F 0 "#PWR023" H 7050 4950 50  0001 C CNN
F 1 "VDD" H 7065 5273 50  0000 C CNN
F 2 "" H 7050 5100 50  0001 C CNN
F 3 "" H 7050 5100 50  0001 C CNN
	1    7050 5100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR025
U 1 1 66C1930F
P 7250 5100
F 0 "#PWR025" H 7250 4950 50  0001 C CNN
F 1 "VDD" H 7265 5273 50  0000 C CNN
F 2 "" H 7250 5100 50  0001 C CNN
F 3 "" H 7250 5100 50  0001 C CNN
	1    7250 5100
	1    0    0    -1  
$EndComp
Text Label 7750 5500 0    50   ~ 0
Audio_Input
Text Label 6650 5500 2    50   ~ 0
CS_DAC
Text Label 6650 5600 2    50   ~ 0
SCK
Text Label 6650 5700 2    50   ~ 0
MOSI
$Comp
L fab:C C7
U 1 1 66C32B7C
P 7650 4950
F 0 "C7" H 7765 4996 50  0000 L CNN
F 1 "C" H 7765 4905 50  0000 L CNN
F 2 "fab:C_1206" H 7688 4800 50  0001 C CNN
F 3 "" H 7650 4950 50  0001 C CNN
	1    7650 4950
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR026
U 1 1 66C33132
P 7650 4800
F 0 "#PWR026" H 7650 4650 50  0001 C CNN
F 1 "VDD" H 7665 4973 50  0000 C CNN
F 2 "" H 7650 4800 50  0001 C CNN
F 3 "" H 7650 4800 50  0001 C CNN
	1    7650 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 66C335C8
P 7650 5100
F 0 "#PWR027" H 7650 4850 50  0001 C CNN
F 1 "GND" V 7655 4972 50  0000 R CNN
F 2 "" H 7650 5100 50  0001 C CNN
F 3 "" H 7650 5100 50  0001 C CNN
	1    7650 5100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR028
U 1 1 66C5A346
P 8050 4950
F 0 "#PWR028" H 8050 4800 50  0001 C CNN
F 1 "VDD" H 8065 5123 50  0000 C CNN
F 2 "" H 8050 4950 50  0001 C CNN
F 3 "" H 8050 4950 50  0001 C CNN
	1    8050 4950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR029
U 1 1 66C5A496
P 8050 4950
F 0 "#PWR029" H 8050 4800 50  0001 C CNN
F 1 "+3V3" V 8065 5078 50  0000 L CNN
F 2 "" H 8050 4950 50  0001 C CNN
F 3 "" H 8050 4950 50  0001 C CNN
	1    8050 4950
	-1   0    0    1   
$EndComp
$Comp
L fab:Regulator_Linear_ZLDO1117-3.3V-1A U1
U 1 1 66D4A544
P 1650 6600
F 0 "U1" H 1650 6842 50  0000 C CNN
F 1 "Regulator_Linear_ZLDO1117-3.3V-1A" H 1650 6751 50  0000 C CNN
F 2 "fab:SOT-223-3_TabPin2" H 1650 6800 50  0001 C CNN
F 3 "http://www.diodes.com/datasheets/AP1117.pdf" H 1750 6350 50  0001 C CNN
	1    1650 6600
	1    0    0    -1  
$EndComp
$Comp
L fab:C C2
U 1 1 66D4A545
P 1350 6750
F 0 "C2" H 1465 6796 50  0000 L CNN
F 1 "C" H 1465 6705 50  0000 L CNN
F 2 "fab:C_1206" H 1388 6600 50  0001 C CNN
F 3 "" H 1350 6750 50  0001 C CNN
	1    1350 6750
	1    0    0    -1  
$EndComp
$Comp
L fab:C C3
U 1 1 6569FF84
P 1950 6750
F 0 "C3" H 2065 6796 50  0000 L CNN
F 1 "C" H 2065 6705 50  0000 L CNN
F 2 "fab:C_1206" H 1988 6600 50  0001 C CNN
F 3 "" H 1950 6750 50  0001 C CNN
	1    1950 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 656A05D1
P 1650 6900
F 0 "#PWR09" H 1650 6650 50  0001 C CNN
F 1 "GND" H 1655 6727 50  0000 C CNN
F 2 "" H 1650 6900 50  0001 C CNN
F 3 "" H 1650 6900 50  0001 C CNN
	1    1650 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 6900 1650 6900
Connection ~ 1650 6900
Wire Wire Line
	1650 6900 1950 6900
$Comp
L power:+3V3 #PWR012
U 1 1 656A0BA3
P 1950 6600
F 0 "#PWR012" H 1950 6450 50  0001 C CNN
F 1 "+3V3" H 1965 6773 50  0000 C CNN
F 2 "" H 1950 6600 50  0001 C CNN
F 3 "" H 1950 6600 50  0001 C CNN
	1    1950 6600
	1    0    0    -1  
$EndComp
Connection ~ 1950 6600
$Comp
L power:+5V #PWR02
U 1 1 656A1307
P 1050 5850
F 0 "#PWR02" H 1050 5700 50  0001 C CNN
F 1 "+5V" H 1065 6023 50  0000 C CNN
F 2 "" H 1050 5850 50  0001 C CNN
F 3 "" H 1050 5850 50  0001 C CNN
	1    1050 5850
	1    0    0    -1  
$EndComp
$Comp
L fab:LED D3
U 1 1 656D149F
P 2050 4600
F 0 "D3" H 2043 4345 50  0000 C CNN
F 1 "LED" H 2043 4436 50  0000 C CNN
F 2 "fab:LED_1206" H 2050 4600 50  0001 C CNN
F 3 "https://optoelectronics.liteon.com/upload/download/DS-22-98-0002/LTST-C150CKT.pdf" H 2050 4600 50  0001 C CNN
	1    2050 4600
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR011
U 1 1 656D1DD8
P 1900 4600
F 0 "#PWR011" H 1900 4450 50  0001 C CNN
F 1 "+3V3" V 1915 4728 50  0000 L CNN
F 2 "" H 1900 4600 50  0001 C CNN
F 3 "" H 1900 4600 50  0001 C CNN
	1    1900 4600
	0    -1   -1   0   
$EndComp
$Comp
L fab:R R2
U 1 1 66D4A54A
P 2350 4600
F 0 "R2" V 2143 4600 50  0000 C CNN
F 1 "R" V 2234 4600 50  0000 C CNN
F 2 "fab:R_1206" V 2280 4600 50  0001 C CNN
F 3 "~" H 2350 4600 50  0001 C CNN
	1    2350 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 66D4A54B
P 2500 4600
F 0 "#PWR013" H 2500 4350 50  0001 C CNN
F 1 "GND" V 2505 4472 50  0000 R CNN
F 2 "" H 2500 4600 50  0001 C CNN
F 3 "" H 2500 4600 50  0001 C CNN
	1    2500 4600
	0    -1   -1   0   
$EndComp
$Comp
L fab:Conn_PinHeader_1x02_P2.54mm_Vertical_THT_D1mm J2
U 1 1 66D3F367
P 6650 4600
F 0 "J2" H 6758 4781 50  0000 C CNN
F 1 "Conn_PinHeader_1x02_P2.54mm_Vertical_THT_D1mm" H 6758 4690 50  0000 C CNN
F 2 "fab:PinHeader_1x02_P2.54mm_Vertical_THT_D1mm" H 6650 4600 50  0001 C CNN
F 3 "~" H 6650 4600 50  0001 C CNN
	1    6650 4600
	1    0    0    -1  
$EndComp
$Comp
L fab:D_Schottky D1
U 1 1 66DCF9E4
P 1050 6000
F 0 "D1" V 1096 5921 50  0000 R CNN
F 1 "D_Schottky" V 1005 5921 50  0000 R CNN
F 2 "fab:SOD-123" H 1050 6000 50  0001 C CNN
F 3 "https://www.st.com/content/ccc/resource/technical/document/datasheet/c6/32/d4/4a/28/d3/4b/11/CD00004930.pdf/files/CD00004930.pdf/jcr:content/translations/en.CD00004930.pdf" H 1050 6000 50  0001 C CNN
	1    1050 6000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1050 6150 1050 6600
Wire Wire Line
	1050 6600 1350 6600
Connection ~ 1350 6600
$Comp
L fab:D_Schottky D2
U 1 1 66DD1EFF
P 1400 6000
F 0 "D2" V 1446 5921 50  0000 R CNN
F 1 "D_Schottky" V 1355 5921 50  0000 R CNN
F 2 "fab:SOD-123" H 1400 6000 50  0001 C CNN
F 3 "https://www.st.com/content/ccc/resource/technical/document/datasheet/c6/32/d4/4a/28/d3/4b/11/CD00004930.pdf/files/CD00004930.pdf/jcr:content/translations/en.CD00004930.pdf" H 1400 6000 50  0001 C CNN
	1    1400 6000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 6150 1350 6150
$Comp
L fab:SWITCH_AYZ0102AGRLC SW2
U 1 1 66DD484A
P 1400 5650
F 0 "SW2" V 1446 5462 50  0000 R CNN
F 1 "SWITCH_AYZ0102AGRLC" V 1355 5462 50  0000 R CNN
F 2 "fab:Switch_SPDT_C&K_AYZ0102AGRLC_7.2x3mm_P2.5mm" H 1400 5650 50  0001 C CNN
F 3 "https://www.ckswitches.com/media/1431/ayz.pdf" H 1400 5650 50  0001 C CNN
	1    1400 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR08
U 1 1 66DD80B0
P 1500 5450
F 0 "#PWR08" H 1500 5300 50  0001 C CNN
F 1 "+BATT" H 1515 5623 50  0000 C CNN
F 2 "" H 1500 5450 50  0001 C CNN
F 3 "" H 1500 5450 50  0001 C CNN
	1    1500 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 66DCDB25
P 1400 3950
F 0 "#PWR07" H 1400 3700 50  0001 C CNN
F 1 "GND" H 1405 3777 50  0000 C CNN
F 2 "" H 1400 3950 50  0001 C CNN
F 3 "" H 1400 3950 50  0001 C CNN
	1    1400 3950
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR06
U 1 1 66DCD49C
P 1400 4250
F 0 "#PWR06" H 1400 4100 50  0001 C CNN
F 1 "+BATT" V 1415 4378 50  0000 L CNN
F 2 "" H 1400 4250 50  0001 C CNN
F 3 "" H 1400 4250 50  0001 C CNN
	1    1400 4250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 66D20170
P 3300 5650
F 0 "#PWR014" H 3300 5400 50  0001 C CNN
F 1 "GND" V 3305 5522 50  0000 R CNN
F 2 "" H 3300 5650 50  0001 C CNN
F 3 "" H 3300 5650 50  0001 C CNN
	1    3300 5650
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 3950 5000 5200
Wire Wire Line
	1350 6150 1350 6600
$Comp
L Device:Battery_Cell BT1
U 1 1 66D47F21
P 1400 4150
F 0 "BT1" H 1518 4246 50  0000 L CNN
F 1 "Battery_Cell" H 1518 4155 50  0000 L CNN
F 2 "New_Library:9V_CONTACT_ON_BRD" V 1400 4210 50  0001 C CNN
F 3 "~" V 1400 4210 50  0001 C CNN
	1    1400 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 6569688D
P 1000 1100
F 0 "#PWR01" H 1000 850 50  0001 C CNN
F 1 "GND" V 1005 972 50  0000 R CNN
F 2 "" H 1000 1100 50  0001 C CNN
F 3 "" H 1000 1100 50  0001 C CNN
	1    1000 1100
	0    1    1    0   
$EndComp
$Comp
L fab:R R1
U 1 1 65698D19
P 1400 950
F 0 "R1" H 1470 996 50  0000 L CNN
F 1 "R" H 1470 905 50  0000 L CNN
F 2 "fab:R_1206" V 1330 950 50  0001 C CNN
F 3 "~" H 1400 950 50  0001 C CNN
	1    1400 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR05
U 1 1 65699906
P 1400 800
F 0 "#PWR05" H 1400 650 50  0001 C CNN
F 1 "+3V3" H 1415 973 50  0000 C CNN
F 2 "" H 1400 800 50  0001 C CNN
F 3 "" H 1400 800 50  0001 C CNN
	1    1400 800 
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Mini J3
U 1 1 6569A51A
P 7000 1000
F 0 "J3" H 6770 897 50  0000 R CNN
F 1 "USB_B_Mini" H 6770 988 50  0000 R CNN
F 2 "Connector_USB:USB_Mini-B_Wuerth_65100516121_Horizontal" H 7150 950 50  0001 C CNN
F 3 "~" H 7150 950 50  0001 C CNN
	1    7000 1000
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 6569C4C1
P 6700 1200
F 0 "#PWR021" H 6700 1050 50  0001 C CNN
F 1 "+5V" V 6715 1328 50  0000 L CNN
F 2 "" H 6700 1200 50  0001 C CNN
F 3 "" H 6700 1200 50  0001 C CNN
	1    6700 1200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 6569CAD9
P 7000 600
F 0 "#PWR022" H 7000 350 50  0001 C CNN
F 1 "GND" H 7005 427 50  0000 C CNN
F 2 "" H 7000 600 50  0001 C CNN
F 3 "" H 7000 600 50  0001 C CNN
	1    7000 600 
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR03
U 1 1 656FE6D2
P 1150 1600
F 0 "#PWR03" H 1150 1450 50  0001 C CNN
F 1 "+3V3" H 1165 1773 50  0000 C CNN
F 2 "" H 1150 1600 50  0001 C CNN
F 3 "" H 1150 1600 50  0001 C CNN
	1    1150 1600
	1    0    0    -1  
$EndComp
$Comp
L fab:C C1
U 1 1 656FE6D8
P 1150 1750
F 0 "C1" H 1265 1796 50  0000 L CNN
F 1 "C" H 1265 1705 50  0000 L CNN
F 2 "fab:C_1206" H 1188 1600 50  0001 C CNN
F 3 "" H 1150 1750 50  0001 C CNN
	1    1150 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 656FE6DE
P 1150 1900
F 0 "#PWR04" H 1150 1650 50  0001 C CNN
F 1 "GND" H 1155 1727 50  0000 C CNN
F 2 "" H 1150 1900 50  0001 C CNN
F 3 "" H 1150 1900 50  0001 C CNN
	1    1150 1900
	1    0    0    -1  
$EndComp
$Comp
L fab:BUTTON_PTS636 SW1
U 1 1 661E479E
P 1200 1100
F 0 "SW1" H 1200 1385 50  0000 C CNN
F 1 "BUTTON_PTS636" H 1200 1294 50  0000 C CNN
F 2 "fab:Button_C&K_PTS636_6x3.5mm" H 1200 1300 50  0001 C CNN
F 3 "https://dznh3ojzb2azq.cloudfront.net/products/Tactile/PTS636/documents/datasheet.pdf" H 1200 1300 50  0001 C CNN
	1    1200 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1100 1400 1100
Connection ~ 1400 1100
Wire Wire Line
	6350 3450 6350 3350
$Comp
L power:GND #PWR019
U 1 1 661FFD22
P 6350 3350
F 0 "#PWR019" H 6350 3100 50  0001 C CNN
F 1 "GND" V 6355 3222 50  0000 R CNN
F 2 "" H 6350 3350 50  0001 C CNN
F 3 "" H 6350 3350 50  0001 C CNN
	1    6350 3350
	0    -1   -1   0   
$EndComp
Connection ~ 6350 3350
Wire Wire Line
	6350 3250 6350 3350
$Comp
L power:+3V3 #PWR010
U 1 1 6620095B
P 1700 3350
F 0 "#PWR010" H 1700 3200 50  0001 C CNN
F 1 "+3V3" H 1715 3523 50  0000 C CNN
F 2 "" H 1700 3350 50  0001 C CNN
F 3 "" H 1700 3350 50  0001 C CNN
	1    1700 3350
	0    -1   -1   0   
$EndComp
Text Label 6350 2200 0    50   ~ 0
MOSI
Text Label 6350 2300 0    50   ~ 0
SCK
Text Label 6350 2400 0    50   ~ 0
MISO
Text Label 6350 1400 0    50   ~ 0
RX
Text Label 6350 1300 0    50   ~ 0
TX
Text Label 1700 2950 2    50   ~ 0
IO18
Text Label 1700 2850 2    50   ~ 0
IO17
Text Label 1700 1250 2    50   ~ 0
IO1
Text Label 1700 1350 2    50   ~ 0
IO2
Text Label 1700 1550 2    50   ~ 0
IO4
Text Label 1700 1650 2    50   ~ 0
IO5
Text Label 1700 1750 2    50   ~ 0
IO6
Text Label 1700 1850 2    50   ~ 0
IO7
Text Label 1700 1950 2    50   ~ 0
IO8
Text Label 1700 2050 2    50   ~ 0
IO9
Text Label 1700 2250 2    50   ~ 0
IO11
Text Label 1700 2350 2    50   ~ 0
IO12
Text Label 1700 2450 2    50   ~ 0
IO13
Text Label 1700 2550 2    50   ~ 0
IO14
Text Label 1700 2650 2    50   ~ 0
IO15
Text Label 1700 2750 2    50   ~ 0
IO16
Text Label 6350 1600 0    50   ~ 0
IO21
Text Label 6350 2000 0    50   ~ 0
IO47
Text Label 6350 1900 0    50   ~ 0
IO48
Text Label 6350 2700 0    50   ~ 0
IO39
Text Label 6350 2800 0    50   ~ 0
IO40
Text Label 6350 2900 0    50   ~ 0
IO41
Text Label 6350 3000 0    50   ~ 0
IO42
Text Label 1700 1450 2    50   ~ 0
IO3
Text Label 6350 1700 0    50   ~ 0
IO45
Text Label 6350 1800 0    50   ~ 0
IO46
Text Label 6350 2500 0    50   ~ 0
IO38
Wire Wire Line
	6350 900  6550 900 
Wire Wire Line
	6550 900  6550 1000
Wire Wire Line
	6550 1000 6700 1000
Wire Wire Line
	6350 1000 6450 1000
Wire Wire Line
	6450 1000 6450 950 
Wire Wire Line
	6450 950  6700 950 
Wire Wire Line
	6700 950  6700 900 
Text Label 1700 2150 2    50   ~ 0
IO10
$Comp
L Connector_Generic:Conn_01x01 J1
U 1 1 66BE3F97
P 1700 800
F 0 "J1" V 1664 712 50  0000 R CNN
F 1 "Conn_01x01" V 1573 712 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 1700 800 50  0001 C CNN
F 3 "~" H 1700 800 50  0001 C CNN
	1    1700 800 
	0    -1   -1   0   
$EndComp
$Comp
L Espressif:ESP32-S3-WROOM-1 U2
U 1 1 661F5613
P 4050 2150
F 0 "U2" H 4025 3665 50  0000 C CNN
F 1 "ESP32-S3-WROOM-1" H 4025 3574 50  0000 C CNN
F 2 "Espressif:ESP32-S3-WROOM-1" H 4050 600 50  0001 C CNN
F 3 "" H 4050 2450 50  0001 C CNN
	1    4050 2150
	1    0    0    -1  
$EndComp
Text Label 7950 2950 2    50   ~ 0
CS_DAC
Text Label 8100 2950 0    50   ~ 0
IO38
Wire Wire Line
	8100 2950 7950 2950
$Comp
L Mechanical:MountingHole H1
U 1 1 66DA7D73
P 3350 6550
F 0 "H1" H 3450 6596 50  0000 L CNN
F 1 "MountingHole" H 3450 6505 50  0000 L CNN
F 2 "New_Library:SPEAKER_MOUNT" H 3350 6550 50  0001 C CNN
F 3 "~" H 3350 6550 50  0001 C CNN
	1    3350 6550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 66DB413C
P 4150 6600
F 0 "H3" H 4250 6646 50  0000 L CNN
F 1 "MountingHole" H 4250 6555 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 4150 6600 50  0001 C CNN
F 3 "~" H 4150 6600 50  0001 C CNN
	1    4150 6600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 66DB47A5
P 4700 6600
F 0 "H4" H 4800 6646 50  0000 L CNN
F 1 "MountingHole" H 4800 6555 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 4700 6600 50  0001 C CNN
F 3 "~" H 4700 6600 50  0001 C CNN
	1    4700 6600
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H5
U 1 1 66DB4E97
P 5300 6650
F 0 "H5" H 5400 6696 50  0000 L CNN
F 1 "MountingHole" H 5400 6605 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 5300 6650 50  0001 C CNN
F 3 "~" H 5300 6650 50  0001 C CNN
	1    5300 6650
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 66DB5E5B
P 3400 7200
F 0 "H2" H 3500 7246 50  0000 L CNN
F 1 "MountingHole" H 3500 7155 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2" H 3400 7200 50  0001 C CNN
F 3 "~" H 3400 7200 50  0001 C CNN
	1    3400 7200
	1    0    0    -1  
$EndComp
$Comp
L Connector:AudioJack3 J6
U 1 1 6493279E
P 7550 2100
F 0 "J6" H 7582 2425 50  0000 C CNN
F 1 "AudioJack2_Ground" H 7582 2334 50  0000 C CNN
F 2 "Connector_Audio:Jack_3.5mm_CUI_SJ-3523-SMT_Horizontal" H 7550 2100 50  0001 C CNN
F 3 "~" H 7550 2100 50  0001 C CNN
	1    7550 2100
	1    0    0    -1  
$EndComp
Text Label 8050 2000 0    50   ~ 0
HdVo1
Text Label 7750 2100 0    50   ~ 0
HdVo2
Text Label 7750 2200 0    50   ~ 0
HdVo2
$Comp
L Switch:SW_Push_DPDT SW3
U 1 1 64119424
P 5650 5800
F 0 "SW3" H 5650 6285 50  0000 C CNN
F 1 "SW_Push_DPDT" H 5650 6194 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_DPDT_CK_JS202011JCQN" H 5650 6000 50  0001 C CNN
F 3 "~" H 5650 6000 50  0001 C CNN
	1    5650 5800
	1    0    0    -1  
$EndComp
Text Label 5450 5600 2    50   ~ 0
Vo1
Text Label 5450 6000 2    50   ~ 0
Vo2
Text Label 5850 5500 0    50   ~ 0
SpVo1
Text Label 5850 5900 0    50   ~ 0
SpVo2
Text Label 5850 5700 0    50   ~ 0
HdVo1
Text Label 5850 6100 0    50   ~ 0
HdVo2
$Comp
L fab:R R10
U 1 1 670695F5
P 7900 2000
F 0 "R10" V 7693 2000 50  0000 C CNN
F 1 "R" V 7784 2000 50  0000 C CNN
F 2 "fab:R_1206" V 7830 2000 50  0001 C CNN
F 3 "~" H 7900 2000 50  0001 C CNN
	1    7900 2000
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 6717FA00
P 1700 650
F 0 "J7" V 1664 562 50  0000 R CNN
F 1 "Conn_01x01" V 1573 562 50  0000 R CNN
F 2 "Connector_Wire:SolderWirePad_1x01_SMD_1x2mm" H 1700 650 50  0001 C CNN
F 3 "~" H 1700 650 50  0001 C CNN
	1    1700 650 
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 671801A2
P 1900 650
F 0 "#PWR0108" H 1900 400 50  0001 C CNN
F 1 "GND" V 1905 522 50  0000 R CNN
F 2 "" H 1900 650 50  0001 C CNN
F 3 "" H 1900 650 50  0001 C CNN
	1    1900 650 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3900 3950 3900 4600
Wire Wire Line
	3900 3950 5000 3950
Text Label 10150 1500 0    50   ~ 0
IO12
Text Label 10150 1800 0    50   ~ 0
IO13
Text Label 10050 4500 2    50   ~ 0
IO14
Text Label 10050 4200 2    50   ~ 0
IO47
Text Label 10050 3750 2    50   ~ 0
IO45
Text Label 10050 3450 2    50   ~ 0
IO39
Text Label 10050 3150 2    50   ~ 0
IO40
Text Label 10050 2850 2    50   ~ 0
IO41
Text Label 10050 2400 2    50   ~ 0
IO42
Text Label 8450 700  0    50   ~ 0
IO4
Text Label 8450 1000 0    50   ~ 0
IO5
Text Label 8450 1300 0    50   ~ 0
IO6
Text Label 8450 1600 0    50   ~ 0
IO7
Text Label 8450 1900 0    50   ~ 0
IO15
Text Label 8450 2650 0    50   ~ 0
IO17
Text Label 8450 2200 0    50   ~ 0
IO16
Text Label 8450 3250 0    50   ~ 0
IO8
Text Label 8450 3550 0    50   ~ 0
IO3
Text Label 8450 4000 0    50   ~ 0
IO46
Text Label 10150 900  0    50   ~ 0
IO10
Text Label 10150 1200 0    50   ~ 0
IO11
$Comp
L Switch:SW_SPDT SW4
U 1 1 66DF49C4
P 9000 650
F 0 "SW4" H 9000 935 50  0000 C CNN
F 1 "SW_SPDT" H 9000 844 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 650 50  0001 C CNN
F 3 "~" H 9000 650 50  0001 C CNN
	1    9000 650 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 66DFF82C
P 9200 750
F 0 "#PWR0105" H 9200 500 50  0001 C CNN
F 1 "GND" V 9205 622 50  0000 R CNN
F 2 "" H 9200 750 50  0001 C CNN
F 3 "" H 9200 750 50  0001 C CNN
	1    9200 750 
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW5
U 1 1 66E00CF2
P 9000 1050
F 0 "SW5" H 9000 1335 50  0000 C CNN
F 1 "SW_SPDT" H 9000 1244 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 1050 50  0001 C CNN
F 3 "~" H 9000 1050 50  0001 C CNN
	1    9000 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 66E00CF8
P 9200 1150
F 0 "#PWR0106" H 9200 900 50  0001 C CNN
F 1 "GND" V 9205 1022 50  0000 R CNN
F 2 "" H 9200 1150 50  0001 C CNN
F 3 "" H 9200 1150 50  0001 C CNN
	1    9200 1150
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW6
U 1 1 66E01C58
P 9000 1500
F 0 "SW6" H 9000 1785 50  0000 C CNN
F 1 "SW_SPDT" H 9000 1694 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 1500 50  0001 C CNN
F 3 "~" H 9000 1500 50  0001 C CNN
	1    9000 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 66E01C5E
P 9200 1600
F 0 "#PWR0107" H 9200 1350 50  0001 C CNN
F 1 "GND" V 9205 1472 50  0000 R CNN
F 2 "" H 9200 1600 50  0001 C CNN
F 3 "" H 9200 1600 50  0001 C CNN
	1    9200 1600
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW7
U 1 1 66E027E4
P 9000 1800
F 0 "SW7" H 9000 2085 50  0000 C CNN
F 1 "SW_SPDT" H 9000 1994 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 1800 50  0001 C CNN
F 3 "~" H 9000 1800 50  0001 C CNN
	1    9000 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 66E027EA
P 9200 1900
F 0 "#PWR0109" H 9200 1650 50  0001 C CNN
F 1 "GND" V 9205 1772 50  0000 R CNN
F 2 "" H 9200 1900 50  0001 C CNN
F 3 "" H 9200 1900 50  0001 C CNN
	1    9200 1900
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW8
U 1 1 66E03279
P 9000 2100
F 0 "SW8" H 9000 2385 50  0000 C CNN
F 1 "SW_SPDT" H 9000 2294 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 2100 50  0001 C CNN
F 3 "~" H 9000 2100 50  0001 C CNN
	1    9000 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 66E0327F
P 9200 2200
F 0 "#PWR0110" H 9200 1950 50  0001 C CNN
F 1 "GND" V 9205 2072 50  0000 R CNN
F 2 "" H 9200 2200 50  0001 C CNN
F 3 "" H 9200 2200 50  0001 C CNN
	1    9200 2200
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW9
U 1 1 66E04AE0
P 9000 2400
F 0 "SW9" H 9000 2685 50  0000 C CNN
F 1 "SW_SPDT" H 9000 2594 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 2400 50  0001 C CNN
F 3 "~" H 9000 2400 50  0001 C CNN
	1    9000 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 66E04AE6
P 9200 2500
F 0 "#PWR0111" H 9200 2250 50  0001 C CNN
F 1 "GND" V 9205 2372 50  0000 R CNN
F 2 "" H 9200 2500 50  0001 C CNN
F 3 "" H 9200 2500 50  0001 C CNN
	1    9200 2500
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW10
U 1 1 66E0574E
P 9000 2850
F 0 "SW10" H 9000 3135 50  0000 C CNN
F 1 "SW_SPDT" H 9000 3044 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 2850 50  0001 C CNN
F 3 "~" H 9000 2850 50  0001 C CNN
	1    9000 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 66E05754
P 9200 2950
F 0 "#PWR0112" H 9200 2700 50  0001 C CNN
F 1 "GND" V 9205 2822 50  0000 R CNN
F 2 "" H 9200 2950 50  0001 C CNN
F 3 "" H 9200 2950 50  0001 C CNN
	1    9200 2950
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW11
U 1 1 66E0627A
P 9000 3150
F 0 "SW11" H 9000 3435 50  0000 C CNN
F 1 "SW_SPDT" H 9000 3344 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 3150 50  0001 C CNN
F 3 "~" H 9000 3150 50  0001 C CNN
	1    9000 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 66E06280
P 9200 3250
F 0 "#PWR0113" H 9200 3000 50  0001 C CNN
F 1 "GND" V 9205 3122 50  0000 R CNN
F 2 "" H 9200 3250 50  0001 C CNN
F 3 "" H 9200 3250 50  0001 C CNN
	1    9200 3250
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW12
U 1 1 66E06C64
P 9000 3450
F 0 "SW12" H 9000 3735 50  0000 C CNN
F 1 "SW_SPDT" H 9000 3644 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 3450 50  0001 C CNN
F 3 "~" H 9000 3450 50  0001 C CNN
	1    9000 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 66E06C6A
P 9200 3550
F 0 "#PWR0114" H 9200 3300 50  0001 C CNN
F 1 "GND" V 9205 3422 50  0000 R CNN
F 2 "" H 9200 3550 50  0001 C CNN
F 3 "" H 9200 3550 50  0001 C CNN
	1    9200 3550
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW13
U 1 1 66E07986
P 9000 3750
F 0 "SW13" H 9000 4035 50  0000 C CNN
F 1 "SW_SPDT" H 9000 3944 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 3750 50  0001 C CNN
F 3 "~" H 9000 3750 50  0001 C CNN
	1    9000 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 66E0798C
P 9200 3850
F 0 "#PWR0115" H 9200 3600 50  0001 C CNN
F 1 "GND" V 9205 3722 50  0000 R CNN
F 2 "" H 9200 3850 50  0001 C CNN
F 3 "" H 9200 3850 50  0001 C CNN
	1    9200 3850
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW14
U 1 1 66E08620
P 9000 4050
F 0 "SW14" H 9000 4335 50  0000 C CNN
F 1 "SW_SPDT" H 9000 4244 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 4050 50  0001 C CNN
F 3 "~" H 9000 4050 50  0001 C CNN
	1    9000 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 66E08626
P 9200 4150
F 0 "#PWR0116" H 9200 3900 50  0001 C CNN
F 1 "GND" V 9205 4022 50  0000 R CNN
F 2 "" H 9200 4150 50  0001 C CNN
F 3 "" H 9200 4150 50  0001 C CNN
	1    9200 4150
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW15
U 1 1 66E09C3A
P 9000 4350
F 0 "SW15" H 9000 4635 50  0000 C CNN
F 1 "SW_SPDT" H 9000 4544 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 9000 4350 50  0001 C CNN
F 3 "~" H 9000 4350 50  0001 C CNN
	1    9000 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 66E09C40
P 9200 4450
F 0 "#PWR0117" H 9200 4200 50  0001 C CNN
F 1 "GND" V 9205 4322 50  0000 R CNN
F 2 "" H 9200 4450 50  0001 C CNN
F 3 "" H 9200 4450 50  0001 C CNN
	1    9200 4450
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW16
U 1 1 66E2A57A
P 10700 850
F 0 "SW16" H 10700 1135 50  0000 C CNN
F 1 "SW_SPDT" H 10700 1044 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 850 50  0001 C CNN
F 3 "~" H 10700 850 50  0001 C CNN
	1    10700 850 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 66E2A580
P 10900 950
F 0 "#PWR0118" H 10900 700 50  0001 C CNN
F 1 "GND" V 10905 822 50  0000 R CNN
F 2 "" H 10900 950 50  0001 C CNN
F 3 "" H 10900 950 50  0001 C CNN
	1    10900 950 
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW17
U 1 1 66E2A586
P 10700 1250
F 0 "SW17" H 10700 1535 50  0000 C CNN
F 1 "SW_SPDT" H 10700 1444 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 1250 50  0001 C CNN
F 3 "~" H 10700 1250 50  0001 C CNN
	1    10700 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 66E2A58C
P 10900 1350
F 0 "#PWR0119" H 10900 1100 50  0001 C CNN
F 1 "GND" V 10905 1222 50  0000 R CNN
F 2 "" H 10900 1350 50  0001 C CNN
F 3 "" H 10900 1350 50  0001 C CNN
	1    10900 1350
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW18
U 1 1 66E2A592
P 10700 1700
F 0 "SW18" H 10700 1985 50  0000 C CNN
F 1 "SW_SPDT" H 10700 1894 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 1700 50  0001 C CNN
F 3 "~" H 10700 1700 50  0001 C CNN
	1    10700 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 66E2A598
P 10900 1800
F 0 "#PWR0120" H 10900 1550 50  0001 C CNN
F 1 "GND" V 10905 1672 50  0000 R CNN
F 2 "" H 10900 1800 50  0001 C CNN
F 3 "" H 10900 1800 50  0001 C CNN
	1    10900 1800
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW19
U 1 1 66E2A59E
P 10700 2000
F 0 "SW19" H 10700 2285 50  0000 C CNN
F 1 "SW_SPDT" H 10700 2194 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 2000 50  0001 C CNN
F 3 "~" H 10700 2000 50  0001 C CNN
	1    10700 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 66E2A5A4
P 10900 2100
F 0 "#PWR0121" H 10900 1850 50  0001 C CNN
F 1 "GND" V 10905 1972 50  0000 R CNN
F 2 "" H 10900 2100 50  0001 C CNN
F 3 "" H 10900 2100 50  0001 C CNN
	1    10900 2100
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW20
U 1 1 66E2A5AA
P 10700 2300
F 0 "SW20" H 10700 2585 50  0000 C CNN
F 1 "SW_SPDT" H 10700 2494 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 2300 50  0001 C CNN
F 3 "~" H 10700 2300 50  0001 C CNN
	1    10700 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 66E2A5B0
P 10900 2400
F 0 "#PWR0122" H 10900 2150 50  0001 C CNN
F 1 "GND" V 10905 2272 50  0000 R CNN
F 2 "" H 10900 2400 50  0001 C CNN
F 3 "" H 10900 2400 50  0001 C CNN
	1    10900 2400
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW21
U 1 1 66E2A5B6
P 10700 2600
F 0 "SW21" H 10700 2885 50  0000 C CNN
F 1 "SW_SPDT" H 10700 2794 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 2600 50  0001 C CNN
F 3 "~" H 10700 2600 50  0001 C CNN
	1    10700 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 66E2A5BC
P 10900 2700
F 0 "#PWR0123" H 10900 2450 50  0001 C CNN
F 1 "GND" V 10905 2572 50  0000 R CNN
F 2 "" H 10900 2700 50  0001 C CNN
F 3 "" H 10900 2700 50  0001 C CNN
	1    10900 2700
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW22
U 1 1 66E2A5C2
P 10700 3050
F 0 "SW22" H 10700 3335 50  0000 C CNN
F 1 "SW_SPDT" H 10700 3244 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 3050 50  0001 C CNN
F 3 "~" H 10700 3050 50  0001 C CNN
	1    10700 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 66E2A5C8
P 10900 3150
F 0 "#PWR0124" H 10900 2900 50  0001 C CNN
F 1 "GND" V 10905 3022 50  0000 R CNN
F 2 "" H 10900 3150 50  0001 C CNN
F 3 "" H 10900 3150 50  0001 C CNN
	1    10900 3150
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW23
U 1 1 66E2A5CE
P 10700 3350
F 0 "SW23" H 10700 3635 50  0000 C CNN
F 1 "SW_SPDT" H 10700 3544 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 3350 50  0001 C CNN
F 3 "~" H 10700 3350 50  0001 C CNN
	1    10700 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 66E2A5D4
P 10900 3450
F 0 "#PWR0125" H 10900 3200 50  0001 C CNN
F 1 "GND" V 10905 3322 50  0000 R CNN
F 2 "" H 10900 3450 50  0001 C CNN
F 3 "" H 10900 3450 50  0001 C CNN
	1    10900 3450
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW24
U 1 1 66E2A5DA
P 10700 3650
F 0 "SW24" H 10700 3935 50  0000 C CNN
F 1 "SW_SPDT" H 10700 3844 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 3650 50  0001 C CNN
F 3 "~" H 10700 3650 50  0001 C CNN
	1    10700 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 66E2A5E0
P 10900 3750
F 0 "#PWR0126" H 10900 3500 50  0001 C CNN
F 1 "GND" V 10905 3622 50  0000 R CNN
F 2 "" H 10900 3750 50  0001 C CNN
F 3 "" H 10900 3750 50  0001 C CNN
	1    10900 3750
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW25
U 1 1 66E2A5E6
P 10700 3950
F 0 "SW25" H 10700 4235 50  0000 C CNN
F 1 "SW_SPDT" H 10700 4144 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 3950 50  0001 C CNN
F 3 "~" H 10700 3950 50  0001 C CNN
	1    10700 3950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 66E2A5EC
P 10900 4050
F 0 "#PWR0127" H 10900 3800 50  0001 C CNN
F 1 "GND" V 10905 3922 50  0000 R CNN
F 2 "" H 10900 4050 50  0001 C CNN
F 3 "" H 10900 4050 50  0001 C CNN
	1    10900 4050
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW26
U 1 1 66E2A5F2
P 10700 4250
F 0 "SW26" H 10700 4535 50  0000 C CNN
F 1 "SW_SPDT" H 10700 4444 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 4250 50  0001 C CNN
F 3 "~" H 10700 4250 50  0001 C CNN
	1    10700 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 66E2A5F8
P 10900 4350
F 0 "#PWR0128" H 10900 4100 50  0001 C CNN
F 1 "GND" V 10905 4222 50  0000 R CNN
F 2 "" H 10900 4350 50  0001 C CNN
F 3 "" H 10900 4350 50  0001 C CNN
	1    10900 4350
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_SPDT SW27
U 1 1 66E2A5FE
P 10700 4550
F 0 "SW27" H 10700 4835 50  0000 C CNN
F 1 "SW_SPDT" H 10700 4744 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10700 4550 50  0001 C CNN
F 3 "~" H 10700 4550 50  0001 C CNN
	1    10700 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 66E2A604
P 10900 4650
F 0 "#PWR0129" H 10900 4400 50  0001 C CNN
F 1 "GND" V 10905 4522 50  0000 R CNN
F 2 "" H 10900 4650 50  0001 C CNN
F 3 "" H 10900 4650 50  0001 C CNN
	1    10900 4650
	0    -1   -1   0   
$EndComp
Text Label 8450 4400 0    50   ~ 0
IO9
Text Label 8450 2950 0    50   ~ 0
IO18
Wire Wire Line
	8450 700  8800 700 
Wire Wire Line
	8800 700  8800 650 
Wire Wire Line
	8450 1000 8800 1000
Wire Wire Line
	8800 1000 8800 1050
Wire Wire Line
	8450 1300 8800 1300
Wire Wire Line
	8800 1300 8800 1500
Wire Wire Line
	8450 1600 8800 1600
Wire Wire Line
	8800 1600 8800 1800
Wire Wire Line
	8450 1900 8800 1900
Wire Wire Line
	8800 1900 8800 2100
Wire Wire Line
	8450 2200 8800 2200
Wire Wire Line
	8800 2200 8800 2400
Wire Wire Line
	8450 2650 8800 2650
Wire Wire Line
	8800 2650 8800 2850
Wire Wire Line
	8450 2950 8800 2950
Wire Wire Line
	8800 2950 8800 3150
Wire Wire Line
	8450 3250 8800 3250
Wire Wire Line
	8800 3250 8800 3450
Wire Wire Line
	8450 3550 8800 3550
Wire Wire Line
	8800 3550 8800 3750
Wire Wire Line
	8450 4000 8800 4000
Wire Wire Line
	8800 4000 8800 4050
Wire Wire Line
	8450 4400 8800 4400
Wire Wire Line
	8800 4400 8800 4350
Wire Wire Line
	10150 900  10500 900 
Wire Wire Line
	10500 900  10500 850 
Wire Wire Line
	10150 1200 10500 1200
Wire Wire Line
	10500 1200 10500 1250
Wire Wire Line
	10150 1500 10500 1500
Wire Wire Line
	10500 1500 10500 1700
Wire Wire Line
	10150 1800 10500 1800
Wire Wire Line
	10500 1800 10500 2000
Wire Wire Line
	10050 4500 9700 4500
Wire Wire Line
	10500 2100 10500 2300
Wire Wire Line
	10050 4200 9700 4200
Wire Wire Line
	10500 2400 10500 2600
Wire Wire Line
	10050 3750 9700 3750
Wire Wire Line
	10500 2850 10500 3050
Wire Wire Line
	10050 3450 9700 3450
Wire Wire Line
	10500 3150 10500 3350
Wire Wire Line
	10050 3150 9700 3150
Wire Wire Line
	10500 3450 10500 3650
Wire Wire Line
	10050 2850 9700 2850
Wire Wire Line
	10500 3750 10500 3950
Wire Wire Line
	10050 2400 9700 2400
Wire Wire Line
	10500 4200 10500 4250
Wire Wire Line
	10500 2400 10300 2400
Wire Wire Line
	10300 2400 10300 2500
Wire Wire Line
	10300 2500 9700 2500
Wire Wire Line
	9700 2500 9700 2400
Wire Wire Line
	10500 2850 10350 2850
Wire Wire Line
	10350 2850 10350 2750
Wire Wire Line
	10350 2750 9700 2750
Wire Wire Line
	9700 2750 9700 2850
Wire Wire Line
	10500 3150 10250 3150
Wire Wire Line
	10250 3150 10250 3250
Wire Wire Line
	10250 3250 9700 3250
Wire Wire Line
	9700 3250 9700 3150
Wire Wire Line
	10500 3450 10250 3450
Wire Wire Line
	10250 3450 10250 3550
Wire Wire Line
	10250 3550 9700 3550
Wire Wire Line
	9700 3550 9700 3450
Wire Wire Line
	9700 3750 9700 3950
Wire Wire Line
	9700 3950 10350 3950
Wire Wire Line
	10350 3950 10350 3750
Wire Wire Line
	10350 3750 10500 3750
Wire Wire Line
	10500 4200 10100 4200
Wire Wire Line
	10100 4200 10100 4300
Wire Wire Line
	10100 4300 9700 4300
Wire Wire Line
	9700 4300 9700 4200
Wire Wire Line
	9700 4500 9700 4700
Wire Wire Line
	9700 4700 10500 4700
Wire Wire Line
	10500 4550 10500 4700
Wire Wire Line
	9700 2100 9700 2000
Wire Wire Line
	10050 2000 9700 2000
Wire Wire Line
	10500 2100 9700 2100
Text Label 10050 2000 2    50   ~ 0
IO2
$Comp
L power:GND #PWR0104
U 1 1 66F79241
P 10350 12350
F 0 "#PWR0104" H 10350 12100 50  0001 C CNN
F 1 "GND" V 10355 12222 50  0000 R CNN
F 2 "" H 10350 12350 50  0001 C CNN
F 3 "" H 10350 12350 50  0001 C CNN
	1    10350 12350
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 66F7923B
P 10350 11850
F 0 "#PWR0103" H 10350 11600 50  0001 C CNN
F 1 "GND" V 10355 11722 50  0000 R CNN
F 2 "" H 10350 11850 50  0001 C CNN
F 3 "" H 10350 11850 50  0001 C CNN
	1    10350 11850
	0    -1   -1   0   
$EndComp
Text Label 9950 12250 2    50   ~ 0
IO48
Text Label 9950 11750 2    50   ~ 0
IO21
$Comp
L Switch:SW_SPDT SW31
U 1 1 66F79233
P 10150 12250
F 0 "SW31" H 10150 12535 50  0000 C CNN
F 1 "SW_SPDT" H 10150 12444 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10150 12250 50  0001 C CNN
F 3 "~" H 10150 12250 50  0001 C CNN
	1    10150 12250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW30
U 1 1 66F7922D
P 10150 11750
F 0 "SW30" H 10150 12035 50  0000 C CNN
F 1 "SW_SPDT" H 10150 11944 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10150 11750 50  0001 C CNN
F 3 "~" H 10150 11750 50  0001 C CNN
	1    10150 11750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 66F6F740
P 10250 11150
F 0 "#PWR0102" H 10250 10900 50  0001 C CNN
F 1 "GND" V 10255 11022 50  0000 R CNN
F 2 "" H 10250 11150 50  0001 C CNN
F 3 "" H 10250 11150 50  0001 C CNN
	1    10250 11150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 66F6F2E4
P 10250 10650
F 0 "#PWR0101" H 10250 10400 50  0001 C CNN
F 1 "GND" V 10255 10522 50  0000 R CNN
F 2 "" H 10250 10650 50  0001 C CNN
F 3 "" H 10250 10650 50  0001 C CNN
	1    10250 10650
	0    -1   -1   0   
$EndComp
Text Label 9850 11050 2    50   ~ 0
MISO
Text Label 9850 10550 2    50   ~ 0
IO1
$Comp
L Switch:SW_SPDT SW29
U 1 1 66F6E7BF
P 10050 11050
F 0 "SW29" H 10050 11335 50  0000 C CNN
F 1 "SW_SPDT" H 10050 11244 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10050 11050 50  0001 C CNN
F 3 "~" H 10050 11050 50  0001 C CNN
	1    10050 11050
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPDT SW28
U 1 1 66F6DCD1
P 10050 10550
F 0 "SW28" H 10050 10835 50  0000 C CNN
F 1 "SW_SPDT" H 10050 10744 50  0000 C CNN
F 2 "New_Library:LIMIT_SWITCH_TE_SAJ" H 10050 10550 50  0001 C CNN
F 3 "~" H 10050 10550 50  0001 C CNN
	1    10050 10550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
