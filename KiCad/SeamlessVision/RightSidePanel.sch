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
Wire Wire Line
	3300 3150 3100 3150
Wire Wire Line
	3300 2800 3300 3150
Wire Wire Line
	2500 2700 2700 2700
Wire Wire Line
	2500 3150 2700 3150
Wire Wire Line
	3850 2900 4000 2900
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5E15E4A2
P 4200 2800
F 0 "J1" H 4280 2842 50  0000 L CNN
F 1 "To_RightPanel" H 4280 2751 50  0000 L CNN
F 2 "Connector_Molex:Molex_PicoBlade_53398-0371_1x03-1MP_P1.25mm_Vertical" H 4200 2800 50  0001 C CNN
F 3 "~" H 4200 2800 50  0001 C CNN
	1    4200 2800
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW2
U 1 1 5E15DB1E
P 2900 3150
F 0 "SW2" H 2900 3385 50  0000 C CNN
F 1 "SW_DOWN" H 2900 3294 50  0000 C CNN
F 2 "SamacSys_Parts:B3FS4002P" H 2900 3150 50  0001 C CNN
F 3 "~" H 2900 3150 50  0001 C CNN
	1    2900 3150
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 5E15D779
P 2900 2700
F 0 "SW1" H 2900 2935 50  0000 C CNN
F 1 "SW_UP" H 2900 2844 50  0000 C CNN
F 2 "SamacSys_Parts:B3FS4002P" H 2900 2700 50  0001 C CNN
F 3 "~" H 2900 2700 50  0001 C CNN
	1    2900 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 2800 4000 2800
Wire Wire Line
	3100 2700 4000 2700
Wire Wire Line
	2500 2700 2500 2600
Wire Wire Line
	2500 3150 2500 3050
Wire Wire Line
	3850 2900 3850 3150
Wire Wire Line
	3850 3150 3650 3150
Wire Wire Line
	3650 3150 3650 3100
$Comp
L power:+3.3V #PWR0101
U 1 1 5E73443E
P 2500 3050
F 0 "#PWR0101" H 2500 2900 50  0001 C CNN
F 1 "+3.3V" H 2515 3223 50  0000 C CNN
F 2 "" H 2500 3050 50  0001 C CNN
F 3 "" H 2500 3050 50  0001 C CNN
	1    2500 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 5E734B5E
P 2500 2600
F 0 "#PWR0102" H 2500 2450 50  0001 C CNN
F 1 "+3.3V" H 2515 2773 50  0000 C CNN
F 2 "" H 2500 2600 50  0001 C CNN
F 3 "" H 2500 2600 50  0001 C CNN
	1    2500 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5E734FEB
P 3650 3100
F 0 "#PWR0103" H 3650 2950 50  0001 C CNN
F 1 "+3.3V" H 3665 3273 50  0000 C CNN
F 2 "" H 3650 3100 50  0001 C CNN
F 3 "" H 3650 3100 50  0001 C CNN
	1    3650 3100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
