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
	5250 3050 5150 3050
$Comp
L power:+3V3 #PWR05
U 1 1 602FB49C
P 5150 2925
F 0 "#PWR05" H 5150 2775 50  0001 C CNN
F 1 "+3V3" H 5165 3098 50  0000 C CNN
F 2 "" H 5150 2925 50  0001 C CNN
F 3 "" H 5150 2925 50  0001 C CNN
	1    5150 2925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 602FBBB9
P 5150 4775
F 0 "#PWR06" H 5150 4525 50  0001 C CNN
F 1 "GND" H 5155 4602 50  0000 C CNN
F 2 "" H 5150 4775 50  0001 C CNN
F 3 "" H 5150 4775 50  0001 C CNN
	1    5150 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4625 5150 4775
Wire Wire Line
	5150 2925 5150 3050
Connection ~ 5150 3050
$Comp
L Device:R R1
U 1 1 602FC5C8
P 4250 3125
F 0 "R1" H 4320 3171 50  0000 L CNN
F 1 "10k" H 4320 3080 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4180 3125 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 4250 3125 50  0001 C CNN
	1    4250 3125
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 602FCC2E
P 4250 3525
F 0 "C1" H 3950 3550 50  0000 L CNN
F 1 "2.2uF" H 3900 3450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4288 3375 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/grm188r61a225k" H 4250 3525 50  0001 C CNN
	1    4250 3525
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 602FD812
P 4250 3900
F 0 "#PWR02" H 4250 3650 50  0001 C CNN
F 1 "GND" H 4255 3727 50  0000 C CNN
F 2 "" H 4250 3900 50  0001 C CNN
F 3 "" H 4250 3900 50  0001 C CNN
	1    4250 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3900 4250 3675
$Comp
L power:+3V3 #PWR01
U 1 1 602FDCB6
P 4250 2900
F 0 "#PWR01" H 4250 2750 50  0001 C CNN
F 1 "+3V3" H 4265 3073 50  0000 C CNN
F 2 "" H 4250 2900 50  0001 C CNN
F 3 "" H 4250 2900 50  0001 C CNN
	1    4250 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2900 4250 2975
$Comp
L Device:LED_ALT D1
U 1 1 602FF683
P 6800 3425
F 0 "D1" H 6793 3170 50  0000 C CNN
F 1 "debug" H 6793 3261 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 6800 3425 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/to-1608bc-mre" H 6800 3425 50  0001 C CNN
	1    6800 3425
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 603003A9
P 7150 3425
F 0 "R3" V 6943 3425 50  0000 C CNN
F 1 "10k" V 7034 3425 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7080 3425 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 7150 3425 50  0001 C CNN
	1    7150 3425
	0    1    1    0   
$EndComp
Wire Wire Line
	6950 3425 7000 3425
$Comp
L power:GND #PWR012
U 1 1 60301413
P 7350 3475
F 0 "#PWR012" H 7350 3225 50  0001 C CNN
F 1 "GND" H 7355 3302 50  0000 C CNN
F 2 "" H 7350 3475 50  0001 C CNN
F 3 "" H 7350 3475 50  0001 C CNN
	1    7350 3475
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3475 7350 3425
Wire Wire Line
	7350 3425 7300 3425
$Comp
L Device:R R7
U 1 1 60304C92
P 7025 4000
F 0 "R7" V 6818 4000 50  0000 C CNN
F 1 "100" V 6909 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6955 4000 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079616" H 7025 4000 50  0001 C CNN
	1    7025 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	7175 4000 7250 4000
$Comp
L Device:R R5
U 1 1 60305559
P 6825 4225
F 0 "R5" H 6755 4179 50  0000 R CNN
F 1 "10k" H 6755 4270 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6755 4225 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 6825 4225 50  0001 C CNN
	1    6825 4225
	-1   0    0    1   
$EndComp
Wire Wire Line
	6825 4075 6825 4000
Wire Wire Line
	6825 4000 6875 4000
$Comp
L power:GND #PWR013
U 1 1 60305DE9
P 6825 4775
F 0 "#PWR013" H 6825 4525 50  0001 C CNN
F 1 "GND" H 6830 4602 50  0000 C CNN
F 2 "" H 6825 4775 50  0001 C CNN
F 3 "" H 6825 4775 50  0001 C CNN
	1    6825 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	6825 4775 6825 4375
$Comp
L Connector_Generic:Conn_01x02 J2
U 1 1 603065D9
P 8500 3950
F 0 "J2" H 8450 4075 50  0000 L CNN
F 1 "load_1" H 8375 3750 50  0000 L CNN
F 2 "uart_key:magnet_conn" H 8500 3950 50  0001 C CNN
F 3 "~" H 8500 3950 50  0001 C CNN
	1    8500 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 3950 8200 3950
Wire Wire Line
	8200 3950 8200 3750
Wire Wire Line
	8200 3750 7725 3750
Wire Wire Line
	7550 3750 7550 3800
Wire Wire Line
	7550 4200 7550 4250
Wire Wire Line
	7550 4250 7725 4250
Wire Wire Line
	8200 4250 8200 4050
Wire Wire Line
	8200 4050 8300 4050
$Comp
L Device:D_Schottky D4
U 1 1 60308AE8
P 7725 4000
F 0 "D4" V 7679 4080 50  0000 L CNN
F 1 "ss16" V 7770 4080 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 7725 4000 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/ss16" H 7725 4000 50  0001 C CNN
	1    7725 4000
	0    1    1    0   
$EndComp
Wire Wire Line
	7725 3850 7725 3750
Connection ~ 7725 3750
Wire Wire Line
	7725 3750 7550 3750
Wire Wire Line
	7725 4150 7725 4250
Connection ~ 7725 4250
Wire Wire Line
	7725 4250 8200 4250
$Comp
L Device:R R8
U 1 1 6031068E
P 7025 5100
F 0 "R8" V 6818 5100 50  0000 C CNN
F 1 "100" V 6909 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6955 5100 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079616" H 7025 5100 50  0001 C CNN
	1    7025 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	7175 5100 7250 5100
$Comp
L Device:R R6
U 1 1 60310695
P 6825 5325
F 0 "R6" H 6755 5279 50  0000 R CNN
F 1 "10k" H 6755 5370 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6755 5325 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 6825 5325 50  0001 C CNN
	1    6825 5325
	-1   0    0    1   
$EndComp
Wire Wire Line
	6825 5175 6825 5100
Wire Wire Line
	6825 5100 6875 5100
$Comp
L power:GND #PWR014
U 1 1 6031069D
P 6825 5875
F 0 "#PWR014" H 6825 5625 50  0001 C CNN
F 1 "GND" H 6830 5702 50  0000 C CNN
F 2 "" H 6825 5875 50  0001 C CNN
F 3 "" H 6825 5875 50  0001 C CNN
	1    6825 5875
	1    0    0    -1  
$EndComp
Wire Wire Line
	6825 5875 6825 5475
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 603106A4
P 7825 4725
F 0 "J3" H 7775 4850 50  0000 L CNN
F 1 "Magnit" H 7700 4500 50  0000 L CNN
F 2 "uart_key:ZH_Conn_02x01" H 7825 4725 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/b2b-zr-lf-sn-2" H 7825 4725 50  0001 C CNN
F 4 "https://www.chipdip.ru/product/zhr-2" H 7825 4725 50  0001 C CNN "Ссылка 2"
F 5 "https://www.chipdip.ru/product/szh-002t-003t-p0.5?from=rec_product" H 7825 4725 50  0001 C CNN "Ссылка 3"
	1    7825 4725
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D5
U 1 1 603106B2
P 7425 4700
F 0 "D5" V 7400 4500 50  0000 L CNN
F 1 "ss16" V 7500 4450 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 7425 4700 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/ss16" H 7425 4700 50  0001 C CNN
	1    7425 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	6825 4000 6575 4000
Connection ~ 6825 4000
Wire Wire Line
	6825 5100 6450 5100
Wire Wire Line
	6450 5100 6450 3525
Wire Wire Line
	6450 3525 5950 3525
Connection ~ 6825 5100
$Comp
L Device:Fuse F1
U 1 1 60312D87
P 8200 4550
F 0 "F1" H 8260 4596 50  0000 L CNN
F 1 "0.2А" H 8260 4505 50  0000 L CNN
F 2 "Fuse:Fuse_1812_4532Metric" V 8130 4550 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/mf-msmf020" H 8200 4550 50  0001 C CNN
	1    8200 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 6031433A
P 8200 4775
F 0 "#PWR019" H 8200 4525 50  0001 C CNN
F 1 "GND" H 8205 4602 50  0000 C CNN
F 2 "" H 8200 4775 50  0001 C CNN
F 3 "" H 8200 4775 50  0001 C CNN
	1    8200 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 4775 8200 4700
Wire Wire Line
	8200 4400 8200 4250
Connection ~ 8200 4250
$Comp
L Device:LED_ALT D2
U 1 1 6031F9E1
P 6450 5325
F 0 "D2" V 6400 5475 50  0000 C CNN
F 1 "magnit" V 6325 5500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 6450 5325 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/to-1608bc-myf" H 6450 5325 50  0001 C CNN
	1    6450 5325
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 6031F9E7
P 6450 5675
F 0 "R2" H 6575 5625 50  0000 C CNN
F 1 "10k" H 6575 5725 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6380 5675 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 6450 5675 50  0001 C CNN
	1    6450 5675
	-1   0    0    1   
$EndComp
Wire Wire Line
	6450 5475 6450 5525
$Comp
L power:GND #PWR010
U 1 1 6031F9EE
P 6450 5875
F 0 "#PWR010" H 6450 5625 50  0001 C CNN
F 1 "GND" H 6455 5702 50  0000 C CNN
F 2 "" H 6450 5875 50  0001 C CNN
F 3 "" H 6450 5875 50  0001 C CNN
	1    6450 5875
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 5875 6450 5825
Wire Wire Line
	6450 5175 6450 5100
Connection ~ 6450 5100
$Comp
L Device:LED_ALT D3
U 1 1 6032654E
P 6575 4225
F 0 "D3" V 6775 4475 50  0000 C CNN
F 1 "load_1" V 6675 4500 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 6575 4225 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/to-1608bc-myf" H 6575 4225 50  0001 C CNN
	1    6575 4225
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 60326554
P 6575 4575
F 0 "R4" H 6825 4550 50  0000 C CNN
F 1 "10k" H 6825 4625 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6505 4575 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 6575 4575 50  0001 C CNN
	1    6575 4575
	-1   0    0    1   
$EndComp
Wire Wire Line
	6575 4375 6575 4425
$Comp
L power:GND #PWR011
U 1 1 6032655B
P 6575 4775
F 0 "#PWR011" H 6575 4525 50  0001 C CNN
F 1 "GND" H 6580 4602 50  0000 C CNN
F 2 "" H 6575 4775 50  0001 C CNN
F 3 "" H 6575 4775 50  0001 C CNN
	1    6575 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	6575 4775 6575 4725
Wire Wire Line
	6575 4075 6575 4000
$Comp
L 74xGxx:74LVC2G17 U1
U 2 1 6032BDB1
P 5275 5500
F 0 "U1" H 5650 5650 50  0000 C CNN
F 1 "74LVC2G17" H 5800 5550 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 5275 5500 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/sn74lvc2g17dckr-3" H 5275 5500 50  0001 C CNN
	2    5275 5500
	1    0    0    -1  
$EndComp
$Comp
L 74xGxx:74LVC2G17 U1
U 1 1 6032CD10
P 5225 5950
F 0 "U1" H 4700 5800 50  0000 C CNN
F 1 "74LVC2G17" H 4550 5900 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 5225 5950 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/sn74lvc2g17dckr-3" H 5225 5950 50  0001 C CNN
	1    5225 5950
	-1   0    0    1   
$EndComp
Wire Wire Line
	5525 5950 6125 5950
Wire Wire Line
	6125 5950 6125 4125
Wire Wire Line
	6125 4125 5950 4125
Wire Wire Line
	5525 5500 6025 5500
Wire Wire Line
	6025 5500 6025 4225
Wire Wire Line
	6025 4225 5950 4225
Wire Wire Line
	4975 5950 4800 5950
Wire Wire Line
	4800 5950 4800 5675
$Comp
L power:GND #PWR04
U 1 1 60338AC6
P 3650 5850
F 0 "#PWR04" H 3650 5600 50  0001 C CNN
F 1 "GND" H 3655 5677 50  0000 C CNN
F 2 "" H 3650 5850 50  0001 C CNN
F 3 "" H 3650 5850 50  0001 C CNN
	1    3650 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5850 3650 5700
Wire Wire Line
	3775 5400 3600 5400
$Comp
L power:+5P #PWR08
U 1 1 603405DA
P 5275 5250
F 0 "#PWR08" H 5275 5100 50  0001 C CNN
F 1 "+5P" H 5290 5423 50  0000 C CNN
F 2 "" H 5275 5250 50  0001 C CNN
F 3 "" H 5275 5250 50  0001 C CNN
	1    5275 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 60342965
P 5425 5700
F 0 "#PWR09" H 5425 5450 50  0001 C CNN
F 1 "GND" H 5550 5625 50  0000 C CNN
F 2 "" H 5425 5700 50  0001 C CNN
F 3 "" H 5425 5700 50  0001 C CNN
	1    5425 5700
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:MAX5035DUSA U3
U 1 1 6034A916
P 5550 1550
F 0 "U3" H 5550 2017 50  0000 C CNN
F 1 "MAX5035DASA" H 5550 1926 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5700 1200 50  0001 L CIN
F 3 "https://www.chipdip.ru/product/max5035dasa" H 5550 1500 50  0001 C CNN
	1    5550 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 6034B413
P 6650 1550
F 0 "L1" V 6750 1725 50  0000 C CNN
F 1 "220uH" V 6749 1550 50  0000 C CNN
F 2 "uart_key:L_10.4x10.4_H4.8" H 6650 1550 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/b82464g4224m" H 6650 1550 50  0001 C CNN
	1    6650 1550
	0    -1   -1   0   
$EndComp
$Comp
L Device:D_Schottky D6
U 1 1 6034C0AB
P 6300 1925
F 0 "D6" V 6254 2005 50  0000 L CNN
F 1 "ss16" V 6345 2005 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 6300 1925 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/ss16" H 6300 1925 50  0001 C CNN
	1    6300 1925
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 60352842
P 6300 2200
F 0 "#PWR021" H 6300 1950 50  0001 C CNN
F 1 "GND" H 6305 2027 50  0000 C CNN
F 2 "" H 6300 2200 50  0001 C CNN
F 3 "" H 6300 2200 50  0001 C CNN
	1    6300 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2200 6300 2075
Wire Wire Line
	5950 1550 6300 1550
Wire Wire Line
	6300 1550 6300 1775
Connection ~ 6300 1550
Wire Wire Line
	6300 1550 6500 1550
Wire Wire Line
	5950 1750 6875 1750
Wire Wire Line
	6875 1750 6875 1550
Wire Wire Line
	6875 1550 6800 1550
$Comp
L Device:C C3
U 1 1 6035DA9D
P 4975 1925
F 0 "C3" H 5090 1971 50  0000 L CNN
F 1 "0.1uF" H 5090 1880 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5013 1775 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/226236018" H 4975 1925 50  0001 C CNN
	1    4975 1925
	1    0    0    -1  
$EndComp
Wire Wire Line
	4975 1775 4975 1750
Wire Wire Line
	4975 1750 5150 1750
$Comp
L power:GND #PWR020
U 1 1 60366932
P 5650 2200
F 0 "#PWR020" H 5650 1950 50  0001 C CNN
F 1 "GND" H 5655 2027 50  0000 C CNN
F 2 "" H 5650 2200 50  0001 C CNN
F 3 "" H 5650 2200 50  0001 C CNN
	1    5650 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 2200 5650 2100
$Comp
L Device:R R10
U 1 1 60369B62
P 4650 1725
F 0 "R10" H 4720 1771 50  0000 L CNN
F 1 "100k" H 4720 1680 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4580 1725 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079760" H 4650 1725 50  0001 C CNN
F 4 "" H 4650 1725 50  0001 C CNN "Ссылка 2"
	1    4650 1725
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 6036A93F
P 4450 1450
F 0 "R9" V 4550 1450 50  0000 C CNN
F 1 "1M" V 4650 1450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4380 1450 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079784" H 4450 1450 50  0001 C CNN
	1    4450 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 1450 4650 1450
Wire Wire Line
	4650 1450 4650 1575
Wire Wire Line
	4600 1450 4650 1450
Connection ~ 4650 1450
Wire Wire Line
	5150 1350 4250 1350
Wire Wire Line
	4250 1350 4250 1450
Wire Wire Line
	4250 1450 4300 1450
$Comp
L Device:C C4
U 1 1 6037EC8C
P 6300 1300
F 0 "C4" H 6415 1346 50  0000 L CNN
F 1 "0.1uF" H 6415 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 1150 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/226236018" H 6300 1300 50  0001 C CNN
	1    6300 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 1450 6300 1550
Wire Wire Line
	6300 1125 6025 1125
Wire Wire Line
	6025 1125 6025 1350
Wire Wire Line
	6025 1350 5950 1350
Wire Wire Line
	6300 1125 6300 1150
Wire Wire Line
	5650 1950 5650 2100
Connection ~ 5650 2100
Wire Wire Line
	5450 1950 5450 2100
Connection ~ 5450 2100
Wire Wire Line
	5450 2100 5650 2100
Wire Wire Line
	4975 2075 4975 2100
Wire Wire Line
	4975 2100 5450 2100
$Comp
L Device:CP_Small C5
U 1 1 603A627A
P 6875 1925
F 0 "C5" H 6963 1971 50  0000 L CNN
F 1 "15uF" H 6963 1880 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-21_Kemet-B" H 6875 1925 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/342478713" H 6875 1925 50  0001 C CNN
	1    6875 1925
	1    0    0    -1  
$EndComp
Wire Wire Line
	6875 1825 6875 1750
Connection ~ 6875 1750
$Comp
L power:GND #PWR022
U 1 1 603AAD64
P 6875 2200
F 0 "#PWR022" H 6875 1950 50  0001 C CNN
F 1 "GND" H 6880 2027 50  0000 C CNN
F 2 "" H 6875 2200 50  0001 C CNN
F 3 "" H 6875 2200 50  0001 C CNN
	1    6875 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6875 2200 6875 2025
$Comp
L Device:CP_Small C2
U 1 1 603AEED6
P 4250 1925
F 0 "C2" H 4338 1971 50  0000 L CNN
F 1 "68uF" H 4338 1880 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_10x10.5" H 4250 1925 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000565738" H 4250 1925 50  0001 C CNN
	1    4250 1925
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1825 4250 1450
Connection ~ 4250 1450
$Comp
L power:GND #PWR015
U 1 1 603B47E0
P 4250 2200
F 0 "#PWR015" H 4250 1950 50  0001 C CNN
F 1 "GND" H 4255 2027 50  0000 C CNN
F 2 "" H 4250 2200 50  0001 C CNN
F 3 "" H 4250 2200 50  0001 C CNN
	1    4250 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2200 4250 2025
$Comp
L power:+48V #PWR0101
U 1 1 603BA227
P 4250 1200
F 0 "#PWR0101" H 4250 1050 50  0001 C CNN
F 1 "+48V" H 4265 1373 50  0000 C CNN
F 2 "" H 4250 1200 50  0001 C CNN
F 3 "" H 4250 1200 50  0001 C CNN
	1    4250 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1200 4250 1350
Connection ~ 4250 1350
$Comp
L power:+12V #PWR025
U 1 1 603C0656
P 6875 1200
F 0 "#PWR025" H 6875 1050 50  0001 C CNN
F 1 "+12V" H 6890 1373 50  0000 C CNN
F 2 "" H 6875 1200 50  0001 C CNN
F 3 "" H 6875 1200 50  0001 C CNN
	1    6875 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6875 1200 6875 1550
Connection ~ 6875 1550
$Comp
L Transistor_FET:IRLML2060 Q2
U 1 1 603CA381
P 7450 5100
F 0 "Q2" H 7250 5000 50  0000 L CNN
F 1 "IRLML2060" H 7125 4875 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7650 5025 50  0001 L CIN
F 3 "https://www.chipdip.ru/product/irlml2060trpbf" H 7450 5100 50  0001 L CNN
	1    7450 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 603D3159
P 7550 5875
F 0 "#PWR024" H 7550 5625 50  0001 C CNN
F 1 "GND" H 7555 5702 50  0000 C CNN
F 2 "" H 7550 5875 50  0001 C CNN
F 3 "" H 7550 5875 50  0001 C CNN
	1    7550 5875
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5875 7550 5300
Wire Wire Line
	7550 4825 7550 4875
Wire Wire Line
	7550 4825 7625 4825
$Comp
L power:+12V #PWR023
U 1 1 603DE32B
P 7550 4475
F 0 "#PWR023" H 7550 4325 50  0001 C CNN
F 1 "+12V" H 7700 4525 50  0000 C CNN
F 2 "" H 7550 4475 50  0001 C CNN
F 3 "" H 7550 4475 50  0001 C CNN
	1    7550 4475
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 4475 7550 4525
Wire Wire Line
	7550 4725 7625 4725
Wire Wire Line
	7425 4850 7425 4875
Wire Wire Line
	7425 4875 7550 4875
Connection ~ 7550 4875
Wire Wire Line
	7550 4875 7550 4900
Wire Wire Line
	7425 4550 7425 4525
Wire Wire Line
	7425 4525 7550 4525
Connection ~ 7550 4525
Wire Wire Line
	7550 4525 7550 4725
$Comp
L Regulator_Linear:XC6206PxxxMR U4
U 1 1 60432467
P 4275 5000
F 0 "U4" H 4275 5242 50  0000 C CNN
F 1 "XC6206PxxxMR" H 4275 5151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4275 5225 50  0001 C CIN
F 3 "https://www.chipdip.ru/product/xc6206p332mr-torex" H 4275 5000 50  0001 C CNN
	1    4275 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3975 5000 3775 5000
Wire Wire Line
	3775 5000 3775 5400
Wire Wire Line
	5275 5400 5275 5325
Wire Wire Line
	5275 5650 5225 5650
Wire Wire Line
	5225 5650 5225 5850
Wire Wire Line
	5275 5600 5275 5650
Wire Wire Line
	5275 5650 5425 5650
Wire Wire Line
	5425 5650 5425 5700
Connection ~ 5275 5650
Wire Wire Line
	5225 6050 5225 6125
Wire Wire Line
	5225 6125 4950 6125
Wire Wire Line
	4950 6125 4950 5400
Wire Wire Line
	4950 5325 5275 5325
Connection ~ 5275 5325
Wire Wire Line
	5275 5325 5275 5250
Wire Wire Line
	3775 5400 4950 5400
Connection ~ 3775 5400
Connection ~ 4950 5400
Wire Wire Line
	4950 5400 4950 5325
$Comp
L power:GND #PWR016
U 1 1 6047791B
P 4275 5850
F 0 "#PWR016" H 4275 5600 50  0001 C CNN
F 1 "GND" H 4280 5677 50  0000 C CNN
F 2 "" H 4275 5850 50  0001 C CNN
F 3 "" H 4275 5850 50  0001 C CNN
	1    4275 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4275 5850 4275 5300
$Comp
L power:+3V3 #PWR017
U 1 1 6047C5DB
P 4675 4900
F 0 "#PWR017" H 4675 4750 50  0001 C CNN
F 1 "+3V3" H 4690 5073 50  0000 C CNN
F 2 "" H 4675 4900 50  0001 C CNN
F 3 "" H 4675 4900 50  0001 C CNN
	1    4675 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4575 5000 4675 5000
Wire Wire Line
	4675 5000 4675 4900
$Comp
L Device:C C6
U 1 1 6048207B
P 4675 5175
F 0 "C6" H 4790 5221 50  0000 L CNN
F 1 "2.2uF" H 4790 5130 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4713 5025 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/grm188r61a225k" H 4675 5175 50  0001 C CNN
	1    4675 5175
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 5025 4675 5000
Connection ~ 4675 5000
$Comp
L power:GND #PWR018
U 1 1 6048748E
P 4675 5850
F 0 "#PWR018" H 4675 5600 50  0001 C CNN
F 1 "GND" H 4680 5677 50  0000 C CNN
F 2 "" H 4675 5850 50  0001 C CNN
F 3 "" H 4675 5850 50  0001 C CNN
	1    4675 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4675 5325 4675 5850
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 60491692
P 3400 4225
F 0 "J4" H 3400 4550 50  0000 C CNN
F 1 "st-link" H 3400 4450 50  0000 C CNN
F 2 "uart_key:ZH_Conn_04x01" H 3400 4225 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/b4b-zr" H 3400 4225 50  0001 C CNN
F 4 "https://www.chipdip.ru/product/zhr-4" H 3400 4225 50  0001 C CNN "Ссылка 2"
F 5 "https://www.chipdip.ru/product/szh-002t-003t-p0.5?from=rec_product" H 3400 4225 50  0001 C CNN "Ссылка 3"
	1    3400 4225
	-1   0    0    -1  
$EndComp
Text Label 3675 4225 0    50   ~ 0
CLK
Text Label 3675 4325 0    50   ~ 0
DIO
Wire Wire Line
	3600 4225 3675 4225
Wire Wire Line
	3600 4325 3675 4325
$Comp
L power:GND #PWR07
U 1 1 6049C727
P 3650 4450
F 0 "#PWR07" H 3650 4200 50  0001 C CNN
F 1 "GND" H 3655 4277 50  0000 C CNN
F 2 "" H 3650 4450 50  0001 C CNN
F 3 "" H 3650 4450 50  0001 C CNN
	1    3650 4450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4450 3650 4425
Wire Wire Line
	3600 4425 3650 4425
$Comp
L power:+3V3 #PWR03
U 1 1 604A713D
P 3650 4075
F 0 "#PWR03" H 3650 3925 50  0001 C CNN
F 1 "+3V3" H 3665 4248 50  0000 C CNN
F 2 "" H 3650 4075 50  0001 C CNN
F 3 "" H 3650 4075 50  0001 C CNN
	1    3650 4075
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 4125 3650 4075
Wire Wire Line
	3600 4125 3650 4125
Text Label 6175 4425 0    50   ~ 0
CLK
Text Label 6175 4325 0    50   ~ 0
DIO
Wire Wire Line
	6175 4325 5950 4325
Wire Wire Line
	5950 4425 6175 4425
$Comp
L Device:R R11
U 1 1 604F1443
P 4650 2050
F 0 "R11" H 4720 2096 50  0000 L CNN
F 1 "39k" H 4720 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4580 2050 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079750" H 4650 2050 50  0001 C CNN
F 4 "" H 4650 2050 50  0001 C CNN "Ссылка 2"
	1    4650 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1900 4650 1875
Wire Wire Line
	4650 2200 4650 2225
Wire Wire Line
	4650 2225 4975 2225
Wire Wire Line
	4975 2225 4975 2100
Connection ~ 4975 2100
$Comp
L MCU_ST_STM32F0:STM32F070F6Px U2
U 1 1 604FF969
P 5350 3825
F 0 "U2" H 5275 3075 50  0000 C CNN
F 1 "STM32F070F6Px" H 5675 3075 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 4750 3125 50  0001 R CNN
F 3 "https://www.chipdip.ru/product/stm32f070f6p6" H 5350 3825 50  0001 C CNN
	1    5350 3825
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 60509569
P 4525 3725
F 0 "R12" H 4300 3775 50  0000 L CNN
F 1 "10k" H 4300 3675 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4455 3725 50  0001 C CNN
F 3 "https://www.chipdip.ru/product0/9000079736" H 4525 3725 50  0001 C CNN
	1    4525 3725
	1    0    0    -1  
$EndComp
Wire Wire Line
	4525 3575 4525 3525
Wire Wire Line
	4525 3525 4650 3525
Wire Wire Line
	5250 3050 5250 3125
Wire Wire Line
	5150 3050 5150 3125
$Comp
L power:GND #PWR026
U 1 1 605102AC
P 4525 3900
F 0 "#PWR026" H 4525 3650 50  0001 C CNN
F 1 "GND" H 4530 3727 50  0000 C CNN
F 2 "" H 4525 3900 50  0001 C CNN
F 3 "" H 4525 3900 50  0001 C CNN
	1    4525 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4525 3900 4525 3875
Wire Wire Line
	4250 3275 4250 3325
Wire Wire Line
	4650 3325 4250 3325
Connection ~ 4250 3325
Wire Wire Line
	4250 3325 4250 3375
Text Notes 3300 5425 2    50   ~ 0
+5V
Text Notes 3300 5525 2    50   ~ 0
Rx
Text Notes 3300 5625 2    50   ~ 0
Tx
Text Notes 3300 5725 2    50   ~ 0
Gnd
$Comp
L Device:Jumper_NO_Small JP1
U 1 1 6030FF99
P 6125 3625
F 0 "JP1" H 6125 3550 50  0000 C CNN
F 1 "9600" H 6125 3450 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 6125 3625 50  0001 C CNN
F 3 "~" H 6125 3625 50  0001 C CNN
	1    6125 3625
	1    0    0    -1  
$EndComp
Wire Wire Line
	6025 3625 5950 3625
$Comp
L power:GND #PWR0102
U 1 1 6031DD24
P 6325 3700
F 0 "#PWR0102" H 6325 3450 50  0001 C CNN
F 1 "GND" H 6330 3527 50  0000 C CNN
F 2 "" H 6325 3700 50  0001 C CNN
F 3 "" H 6325 3700 50  0001 C CNN
	1    6325 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6325 3700 6325 3625
Wire Wire Line
	6325 3625 6225 3625
Wire Wire Line
	6650 3425 5950 3425
Text Label 6575 3750 0    50   ~ 0
load_1
Wire Wire Line
	6575 3750 6575 4000
Connection ~ 6575 4000
Text Label 4575 4225 2    50   ~ 0
load_1
Wire Wire Line
	4575 4225 4650 4225
$Comp
L Transistor_FET:IRLML2060 Q1
U 1 1 603D8C05
P 7450 4000
F 0 "Q1" H 7250 3900 50  0000 L CNN
F 1 "IRF630NSPBF" H 7050 3775 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:TO-263-2" H 7650 3925 50  0001 L CIN
F 3 "https://www.chipdip.ru/product/irf630nspbf" H 7450 4000 50  0001 L CNN
	1    7450 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J1
U 1 1 6033F1F7
P 3400 5500
F 0 "J1" H 3350 5925 50  0000 L CNN
F 1 "UART" H 3300 5825 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical" H 3400 5500 50  0001 C CNN
F 3 "https://www.chipdip.ru/product/b5b-xh-a" H 3400 5500 50  0001 C CNN
	1    3400 5500
	-1   0    0    -1  
$EndComp
$Comp
L power:+48V #PWR0103
U 1 1 60347BE1
P 3650 5250
F 0 "#PWR0103" H 3650 5100 50  0001 C CNN
F 1 "+48V" H 3625 5475 50  0000 C CNN
F 2 "" H 3650 5250 50  0001 C CNN
F 3 "" H 3650 5250 50  0001 C CNN
	1    3650 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 5300 3650 5300
Wire Wire Line
	3650 5300 3650 5250
Text Notes 3300 5325 2    50   ~ 0
+48V
Wire Wire Line
	3600 5700 3650 5700
Wire Wire Line
	4125 5500 4125 5600
Wire Wire Line
	4125 5600 3600 5600
Wire Wire Line
	4125 5500 4975 5500
Wire Wire Line
	4800 5675 3850 5675
Wire Wire Line
	3850 5675 3850 5500
Wire Wire Line
	3850 5500 3600 5500
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 60394038
P 8550 5500
F 0 "J5" V 8514 5412 50  0000 R CNN
F 1 "M2" V 8423 5412 50  0000 R CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_ISO7380_Pad_TopBottom" H 8550 5500 50  0001 C CNN
F 3 "~" H 8550 5500 50  0001 C CNN
	1    8550 5500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR027
U 1 1 603951C3
P 8550 5750
F 0 "#PWR027" H 8550 5500 50  0001 C CNN
F 1 "GND" H 8555 5577 50  0000 C CNN
F 2 "" H 8550 5750 50  0001 C CNN
F 3 "" H 8550 5750 50  0001 C CNN
	1    8550 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 5750 8550 5700
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 603A6114
P 8825 5500
F 0 "J6" V 8789 5412 50  0000 R CNN
F 1 "M2" V 8698 5412 50  0000 R CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_ISO7380_Pad_TopBottom" H 8825 5500 50  0001 C CNN
F 3 "~" H 8825 5500 50  0001 C CNN
	1    8825 5500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR028
U 1 1 603A611A
P 8825 5750
F 0 "#PWR028" H 8825 5500 50  0001 C CNN
F 1 "GND" H 8830 5577 50  0000 C CNN
F 2 "" H 8825 5750 50  0001 C CNN
F 3 "" H 8825 5750 50  0001 C CNN
	1    8825 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8825 5750 8825 5700
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 603AD4F1
P 9125 5500
F 0 "J7" V 9089 5412 50  0000 R CNN
F 1 "M2" V 8998 5412 50  0000 R CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_ISO7380_Pad_TopBottom" H 9125 5500 50  0001 C CNN
F 3 "~" H 9125 5500 50  0001 C CNN
	1    9125 5500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR029
U 1 1 603AD4F7
P 9125 5750
F 0 "#PWR029" H 9125 5500 50  0001 C CNN
F 1 "GND" H 9130 5577 50  0000 C CNN
F 2 "" H 9125 5750 50  0001 C CNN
F 3 "" H 9125 5750 50  0001 C CNN
	1    9125 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 5750 9125 5700
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 603BC29B
P 9400 5500
F 0 "J8" V 9364 5412 50  0000 R CNN
F 1 "M2" V 9273 5412 50  0000 R CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_ISO7380_Pad_TopBottom" H 9400 5500 50  0001 C CNN
F 3 "~" H 9400 5500 50  0001 C CNN
	1    9400 5500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 603BC2A1
P 9400 5750
F 0 "#PWR030" H 9400 5500 50  0001 C CNN
F 1 "GND" H 9405 5577 50  0000 C CNN
F 2 "" H 9400 5750 50  0001 C CNN
F 3 "" H 9400 5750 50  0001 C CNN
	1    9400 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 5750 9400 5700
$EndSCHEMATC