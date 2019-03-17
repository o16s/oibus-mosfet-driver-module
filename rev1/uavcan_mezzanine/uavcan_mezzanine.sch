EESchema Schematic File Version 4
LIBS:uavcan_mezzanine-cache
EELAYER 26 0
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
L Connector:Conn_01x04_Female J2
U 1 1 5C47507E
P 6750 2600
F 0 "J2" H 6777 2576 50  0000 L CNN
F 1 "Conn_01x04_Female" H 6777 2485 50  0000 L CNN
F 2 "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical" H 6750 2600 50  0001 C CNN
F 3 "~" H 6750 2600 50  0001 C CNN
	1    6750 2600
	1    0    0    -1  
$EndComp
Text GLabel 6550 2500 0    50   Input ~ 0
VBus
Text GLabel 6550 2600 0    50   Input ~ 0
CANH
Text GLabel 6550 2700 0    50   Input ~ 0
CANL
Text GLabel 6550 2800 0    50   Input ~ 0
GND
$Comp
L Connector:Conn_01x04_Female J3
U 1 1 5C47531B
P 6750 3200
F 0 "J3" H 6777 3176 50  0000 L CNN
F 1 "Conn_01x04_Female" H 6777 3085 50  0000 L CNN
F 2 "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical" H 6750 3200 50  0001 C CNN
F 3 "~" H 6750 3200 50  0001 C CNN
	1    6750 3200
	1    0    0    -1  
$EndComp
Text GLabel 6550 3100 0    50   Input ~ 0
VBus
Text GLabel 6550 3200 0    50   Input ~ 0
CANH
Text GLabel 6550 3300 0    50   Input ~ 0
CANL
Text GLabel 6550 3400 0    50   Input ~ 0
GND
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5C4755E7
P 4450 3400
F 0 "J1" H 4556 3578 50  0000 C CNN
F 1 "Conn_01x02_Male" H 4556 3487 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4450 3400 50  0001 C CNN
F 3 "~" H 4450 3400 50  0001 C CNN
	1    4450 3400
	1    0    0    -1  
$EndComp
Text GLabel 4650 3400 2    50   Input ~ 0
CANL
Text GLabel 4650 3500 2    50   Input ~ 0
CANH
$Comp
L Connector:TestPoint TP1
U 1 1 5C4DE2C5
P 4700 2650
F 0 "TP1" H 4758 2770 50  0000 L CNN
F 1 "TestPoint" H 4758 2679 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 4900 2650 50  0001 C CNN
F 3 "~" H 4900 2650 50  0001 C CNN
	1    4700 2650
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5C4DE39D
P 5200 2650
F 0 "TP2" H 5258 2770 50  0000 L CNN
F 1 "TestPoint" H 5258 2679 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_1.0x1.0mm" H 5400 2650 50  0001 C CNN
F 3 "~" H 5400 2650 50  0001 C CNN
	1    5200 2650
	1    0    0    -1  
$EndComp
Text GLabel 4650 2800 0    50   Input ~ 0
VBus
Text GLabel 5300 2800 2    50   Input ~ 0
GND
Wire Wire Line
	5200 2650 5200 2800
Wire Wire Line
	5200 2800 5300 2800
Wire Wire Line
	4700 2650 4700 2800
Wire Wire Line
	4700 2800 4650 2800
$EndSCHEMATC
