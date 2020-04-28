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
L Connector_Generic:Conn_01x06 J1
U 1 1 5E440AE6
P 2250 3800
F 0 "J1" H 2168 3275 50  0000 C CNN
F 1 "Conn_01x06" H 2168 3366 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_2x03_P2.00mm_Vertical_SMD" H 2250 3800 50  0001 C CNN
F 3 "~" H 2250 3800 50  0001 C CNN
	1    2250 3800
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5E44174B
P 2650 3500
F 0 "TP1" H 2708 3572 50  0000 L CNN
F 1 "TestPoint" H 2708 3527 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 2850 3500 50  0001 C CNN
F 3 "~" H 2850 3500 50  0001 C CNN
	1    2650 3500
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5E44263A
P 2900 3600
F 0 "TP2" H 2958 3672 50  0000 L CNN
F 1 "TestPoint" H 2958 3627 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3100 3600 50  0001 C CNN
F 3 "~" H 3100 3600 50  0001 C CNN
	1    2900 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5E4437C2
P 3150 3700
F 0 "TP3" H 3208 3772 50  0000 L CNN
F 1 "TestPoint" H 3208 3727 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3350 3700 50  0001 C CNN
F 3 "~" H 3350 3700 50  0001 C CNN
	1    3150 3700
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5E444365
P 3400 3800
F 0 "TP4" H 3458 3872 50  0000 L CNN
F 1 "TestPoint" H 3458 3827 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3600 3800 50  0001 C CNN
F 3 "~" H 3600 3800 50  0001 C CNN
	1    3400 3800
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP5
U 1 1 5E4449A4
P 3650 3900
F 0 "TP5" H 3708 3972 50  0000 L CNN
F 1 "TestPoint" H 3708 3927 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 3850 3900 50  0001 C CNN
F 3 "~" H 3850 3900 50  0001 C CNN
	1    3650 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP6
U 1 1 5E444E52
P 3900 4000
F 0 "TP6" H 3958 4072 50  0000 L CNN
F 1 "TestPoint" H 3958 4027 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 4100 4000 50  0001 C CNN
F 3 "~" H 4100 4000 50  0001 C CNN
	1    3900 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3500 2650 3500
Wire Wire Line
	2900 3600 2450 3600
Wire Wire Line
	2450 3700 3150 3700
Wire Wire Line
	3400 3800 2450 3800
Wire Wire Line
	2450 3900 3650 3900
Wire Wire Line
	3900 4000 2450 4000
$EndSCHEMATC