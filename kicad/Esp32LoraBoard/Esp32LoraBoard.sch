EESchema Schematic File Version 4
LIBS:Esp32LoraBoard-cache
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "EspLoraLoard"
Date "31.03.2020"
Rev "0.2"
Comp "Viktor Rees"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:ESP32-WROOM-32 U?
U 1 1 5E764E61
P 6500 5300
F 0 "U?" H 6500 6881 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 6500 6790 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 6500 3800 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 6200 5350 50  0001 C CNN
	1    6500 5300
	1    0    0    -1  
$EndComp
$Comp
L RF_Module:RFM95W-868S2 U?
U 1 1 5E768185
P 8000 6750
F 0 "U?" H 8000 7431 50  0000 C CNN
F 1 "RFM95W-868S2" H 8000 7340 50  0000 C CNN
F 2 "" H 4700 8400 50  0001 C CNN
F 3 "https://www.hoperf.com/data/upload/portal/20181127/5bfcbea20e9ef.pdf" H 4700 8400 50  0001 C CNN
	1    8000 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E768A8E
P 5450 7750
F 0 "#PWR?" H 5450 7500 50  0001 C CNN
F 1 "GND" H 5455 7577 50  0000 C CNN
F 2 "" H 5450 7750 50  0001 C CNN
F 3 "" H 5450 7750 50  0001 C CNN
	1    5450 7750
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT?
U 1 1 5E76B5A2
P 4050 4400
F 0 "BT?" H 4168 4496 50  0000 L CNN
F 1 "LiFoPo-BAtterie" H 4168 4405 50  0000 L CNN
F 2 "" V 4050 4460 50  0001 C CNN
F 3 "~" V 4050 4460 50  0001 C CNN
	1    4050 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5E76C9E8
P 5100 3050
F 0 "#PWR?" H 5100 2900 50  0001 C CNN
F 1 "+3.3V" H 5115 3223 50  0000 C CNN
F 2 "" H 5100 3050 50  0001 C CNN
F 3 "" H 5100 3050 50  0001 C CNN
	1    5100 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J?
U 1 1 5E770A21
P 1450 4250
F 0 "J?" H 1507 4717 50  0000 C CNN
F 1 "USB_B_Micro" H 1507 4626 50  0000 C CNN
F 2 "" H 1600 4200 50  0001 C CNN
F 3 "~" H 1600 4200 50  0001 C CNN
	1    1450 4250
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LD1117S33TR_SOT223 U?
U 1 1 5E771C19
P 2800 4050
F 0 "U?" H 2800 4292 50  0000 C CNN
F 1 "LD1117S33TR_SOT223" H 2800 4201 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2800 4250 50  0001 C CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000544.pdf" H 2900 3800 50  0001 C CNN
	1    2800 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5E774A1D
P 2250 4300
F 0 "C?" H 2365 4346 50  0000 L CNN
F 1 "100nF" H 2365 4255 50  0000 L CNN
F 2 "" H 2288 4150 50  0001 C CNN
F 3 "~" H 2250 4300 50  0001 C CNN
	1    2250 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C?
U 1 1 5E775400
P 3300 4300
F 0 "C?" H 3388 4346 50  0000 L CNN
F 1 "10uF" H 3388 4255 50  0000 L CNN
F 2 "" H 3300 4300 50  0001 C CNN
F 3 "~" H 3300 4300 50  0001 C CNN
	1    3300 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4150 2250 4050
Wire Wire Line
	1750 4050 2250 4050
Connection ~ 2250 4050
Wire Wire Line
	2250 4050 2500 4050
Wire Wire Line
	2250 4450 2250 4700
Wire Wire Line
	2250 4700 2800 4700
Wire Wire Line
	2800 4700 2800 4350
Connection ~ 2800 4700
Wire Wire Line
	3300 4200 3300 4050
Wire Wire Line
	3300 4050 3100 4050
Wire Wire Line
	3300 4700 3300 4400
Wire Wire Line
	2800 4700 3300 4700
Wire Wire Line
	1450 4650 1450 4700
Wire Wire Line
	1450 4700 2250 4700
Connection ~ 2250 4700
Wire Wire Line
	5450 6650 5450 7150
Wire Wire Line
	3300 4700 3300 7150
Wire Wire Line
	3300 7150 5450 7150
Connection ~ 3300 4700
Connection ~ 5450 7150
Wire Wire Line
	5450 7150 5450 7750
Wire Wire Line
	3300 4700 4050 4700
Wire Wire Line
	4050 4700 4050 4500
Wire Wire Line
	4050 4200 4050 4050
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 5E7B10B6
P 3700 3850
F 0 "J?" V 3762 3994 50  0000 L CNN
F 1 "Conn_01x03_Male" V 3853 3994 50  0000 L CNN
F 2 "" H 3700 3850 50  0001 C CNN
F 3 "~" H 3700 3850 50  0001 C CNN
	1    3700 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 4050 3600 4050
Connection ~ 3300 4050
Wire Wire Line
	3800 4050 4050 4050
Wire Wire Line
	3700 4050 3700 3300
Wire Wire Line
	3700 3300 5100 3300
Wire Wire Line
	6500 3300 6500 3900
Wire Wire Line
	5100 3050 5100 3300
Connection ~ 5100 3300
Wire Wire Line
	5100 3300 6500 3300
$EndSCHEMATC
