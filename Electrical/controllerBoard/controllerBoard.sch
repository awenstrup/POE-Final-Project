EESchema Schematic File Version 4
LIBS:controllerBoard-cache
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
L formula:Crystal_SMD Y1
U 1 1 5DB86393
P 1250 4275
F 0 "Y1" H 1200 4575 50  0000 L CNN
F 1 "Crystal_SMD" H 1050 4475 50  0000 L CNN
F 2 "footprints:Crystal_SMD_FA238" H 1200 4350 50  0001 C CNN
F 3 "http://www.txccorp.com/download/products/quartz_crystals/2015TXC_7M_17.pdf" H 1300 4450 50  0001 C CNN
F 4 "DK" H 1250 4275 60  0001 C CNN "MFN"
F 5 "887-1125-1-ND" H 1250 4275 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/txc-corporation/7M-16.000MAAJ-T/887-1125-1-ND/2119014" H 1700 4850 60  0001 C CNN "PurchasingLink"
	1    1250 4275
	1    0    0    -1  
$EndComp
$Comp
L formula:C_30pF C2
U 1 1 5DB865EE
P 1050 4575
F 0 "C2" V 1000 4425 50  0000 L CNN
F 1 "C_30pF" V 1000 4625 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 1088 4425 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Samsung%20PDFs/CL_Series_MLCC_ds.pdf" H 1075 4675 50  0001 C CNN
F 4 "DK" H 1050 4575 60  0001 C CNN "MFN"
F 5 "1276-1130-1-ND" H 1050 4575 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics-america-inc/CL21C300JBANNNC/1276-1130-1-ND/3889216" H 1475 5075 60  0001 C CNN "PurchasingLink"
	1    1050 4575
	1    0    0    -1  
$EndComp
$Comp
L formula:C_30pF C8
U 1 1 5DB8669D
P 1450 4575
F 0 "C8" V 1400 4425 50  0000 L CNN
F 1 "C_30pF" V 1400 4625 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 1488 4425 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Samsung%20PDFs/CL_Series_MLCC_ds.pdf" H 1475 4675 50  0001 C CNN
F 4 "DK" H 1450 4575 60  0001 C CNN "MFN"
F 5 "1276-1130-1-ND" H 1450 4575 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics-america-inc/CL21C300JBANNNC/1276-1130-1-ND/3889216" H 1875 5075 60  0001 C CNN "PurchasingLink"
	1    1450 4575
	1    0    0    -1  
$EndComp
$Comp
L formula:R_10K R15
U 1 1 5DB86749
P 2850 2500
F 0 "R15" V 2850 2500 50  0000 C CNN
F 1 "R_10K" V 2800 2750 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 2780 2500 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 2930 2500 50  0001 C CNN
F 4 "DK" H 2850 2500 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 2850 2500 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 3330 2900 60  0001 C CNN "PurchasingLink"
	1    2850 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 4275 1450 4275
Wire Wire Line
	1450 4275 1450 4325
Wire Wire Line
	1150 4275 1050 4275
Wire Wire Line
	1050 4275 1050 4325
$Comp
L power:GND #PWR04
U 1 1 5DB86B33
P 1050 4725
F 0 "#PWR04" H 1050 4475 50  0001 C CNN
F 1 "GND" H 1055 4552 50  0000 C CNN
F 2 "" H 1050 4725 50  0001 C CNN
F 3 "" H 1050 4725 50  0001 C CNN
	1    1050 4725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5DB86B67
P 1450 4725
F 0 "#PWR010" H 1450 4475 50  0001 C CNN
F 1 "GND" H 1455 4552 50  0000 C CNN
F 2 "" H 1450 4725 50  0001 C CNN
F 3 "" H 1450 4725 50  0001 C CNN
	1    1450 4725
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5DB86B9B
P 1250 4575
F 0 "#PWR06" H 1250 4325 50  0001 C CNN
F 1 "GND" H 1255 4402 50  0000 C CNN
F 2 "" H 1250 4575 50  0001 C CNN
F 3 "" H 1250 4575 50  0001 C CNN
	1    1250 4575
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4425 1250 4575
$Comp
L power:GND #PWR012
U 1 1 5DB86C2E
P 1550 4175
F 0 "#PWR012" H 1550 3925 50  0001 C CNN
F 1 "GND" H 1650 4175 50  0000 C CNN
F 2 "" H 1550 4175 50  0001 C CNN
F 3 "" H 1550 4175 50  0001 C CNN
	1    1550 4175
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4125 1550 4175
Wire Wire Line
	1250 4125 1550 4125
Text Label 2450 1600 0    50   ~ 0
XTAL1
Text Label 2450 1700 0    50   ~ 0
XTAL2
Text Label 850  4325 2    50   ~ 0
XTAL1
Text Label 1750 4325 0    50   ~ 0
XTAL2
Wire Wire Line
	850  4325 1050 4325
Connection ~ 1050 4325
Wire Wire Line
	1050 4325 1050 4425
Wire Wire Line
	1750 4325 1450 4325
Connection ~ 1450 4325
Wire Wire Line
	1450 4325 1450 4425
Text Label 3050 2625 0    50   ~ 0
RESET
Wire Wire Line
	3000 2500 3250 2500
Text Label 2450 1300 0    50   ~ 0
MOSI
Text Label 2450 1400 0    50   ~ 0
MISO
Text Label 2450 1500 0    50   ~ 0
SCK
Wire Wire Line
	750  2300 750  1300
Wire Wire Line
	750  1000 850  1000
Wire Wire Line
	750  2600 750  3100
Wire Wire Line
	750  3100 850  3100
Wire Wire Line
	750  3100 750  3700
Connection ~ 750  3100
Wire Wire Line
	850  1300 750  1300
Connection ~ 750  1300
Wire Wire Line
	750  1300 750  1000
$Comp
L power:GND #PWR02
U 1 1 5DB88CD6
P 750 3700
F 0 "#PWR02" H 750 3450 50  0001 C CNN
F 1 "GND" H 755 3527 50  0000 C CNN
F 2 "" H 750 3700 50  0001 C CNN
F 3 "" H 750 3700 50  0001 C CNN
	1    750  3700
	1    0    0    -1  
$EndComp
NoConn ~ 850  1750
$Comp
L formula:0.1in_Female_Socket_01x16 J2
U 1 1 5DC1BC16
P 10525 1575
F 0 "J2" H 10525 650 50  0000 C CNN
F 1 "0.1in_Female_Socket_01x16" V 10650 1525 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x16_P2.54mm_Vertical" H 10425 2275 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Sullins%20PDFs/Female_Headers.100_DS.pdf" H 10525 2375 50  0001 C CNN
F 4 "Sullins Connector Solutions" H 10625 2475 50  0001 C CNN "MF"
F 5 "PPTC161LFBN-RC" H 10725 2575 50  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/sullins-connector-solutions/PPTC161LFBN-RC/S7014-ND/810154" H 10825 2675 50  0001 C CNN "PurchasingLink"
	1    10525 1575
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR05
U 1 1 5DC1DA23
P 1075 7175
F 0 "#PWR05" H 1075 7025 50  0001 C CNN
F 1 "+3V3" V 1075 7400 50  0000 C CNN
F 2 "" H 1075 7175 50  0001 C CNN
F 3 "" H 1075 7175 50  0001 C CNN
	1    1075 7175
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5DC1DD77
P 875 7175
F 0 "#FLG02" H 875 7250 50  0001 C CNN
F 1 "PWR_FLAG" V 875 7500 50  0000 C CNN
F 2 "" H 875 7175 50  0001 C CNN
F 3 "~" H 875 7175 50  0001 C CNN
	1    875  7175
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5DC1DDDF
P 1075 7175
F 0 "#FLG03" H 1075 7250 50  0001 C CNN
F 1 "PWR_FLAG" V 1075 7500 50  0000 C CNN
F 2 "" H 1075 7175 50  0001 C CNN
F 3 "~" H 1075 7175 50  0001 C CNN
	1    1075 7175
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG04
U 1 1 5DC1DE47
P 1275 7175
F 0 "#FLG04" H 1275 7250 50  0001 C CNN
F 1 "PWR_FLAG" V 1275 7500 50  0000 C CNN
F 2 "" H 1275 7175 50  0001 C CNN
F 3 "~" H 1275 7175 50  0001 C CNN
	1    1275 7175
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5DC1DF0A
P 1275 7175
F 0 "#PWR07" H 1275 6925 50  0001 C CNN
F 1 "GND" V 1275 6975 50  0000 C CNN
F 2 "" H 1275 7175 50  0001 C CNN
F 3 "" H 1275 7175 50  0001 C CNN
	1    1275 7175
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  800  750  1000
Connection ~ 750  1000
NoConn ~ 850  1950
NoConn ~ 850  2050
$Comp
L formula:R_100 R32
U 1 1 5DC24ACA
P 2600 1900
F 0 "R32" V 2600 1900 50  0000 C CNN
F 1 "R_0" V 2550 2075 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 2050 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 2400 50  0001 C CNN
F 4 "DK" H 2600 1900 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 2150 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 2300 60  0001 C CNN "PurchasingLink"
	1    2600 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 1900 2950 1900
$Comp
L formula:R_100 R33
U 1 1 5DC25015
P 2600 2000
F 0 "R33" V 2600 2000 50  0000 C CNN
F 1 "R_0" V 2550 2175 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 2150 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 2500 50  0001 C CNN
F 4 "DK" H 2600 2000 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 2250 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 2400 60  0001 C CNN "PurchasingLink"
	1    2600 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2000 2950 2000
$Comp
L formula:R_100 R34
U 1 1 5DC253EB
P 2600 2100
F 0 "R34" V 2600 2100 50  0000 C CNN
F 1 "R_0" V 2550 2275 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 2250 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 2600 50  0001 C CNN
F 4 "DK" H 2600 2100 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 2350 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 2500 60  0001 C CNN "PurchasingLink"
	1    2600 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2100 2950 2100
$Comp
L formula:R_100 R6
U 1 1 5DC257E8
P 2600 2200
F 0 "R6" V 2600 2200 50  0000 C CNN
F 1 "R_0" V 2550 2375 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 2350 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 2700 50  0001 C CNN
F 4 "DK" H 2600 2200 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 2450 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 2600 60  0001 C CNN "PurchasingLink"
	1    2600 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2200 2950 2200
$Comp
L formula:R_100 R35
U 1 1 5DC26B06
P 2600 2800
F 0 "R35" V 2600 2800 50  0000 C CNN
F 1 "R_0" V 2550 2975 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 2950 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3300 50  0001 C CNN
F 4 "DK" H 2600 2800 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3050 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3200 60  0001 C CNN "PurchasingLink"
	1    2600 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2800 2950 2800
$Comp
L formula:R_100 R10
U 1 1 5DC27059
P 2600 2900
F 0 "R10" V 2600 2900 50  0000 C CNN
F 1 "R_0" V 2550 3075 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 3050 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3400 50  0001 C CNN
F 4 "DK" H 2600 2900 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3150 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3300 60  0001 C CNN "PurchasingLink"
	1    2600 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2900 2950 2900
$Comp
L formula:R_100 R11
U 1 1 5DC275E7
P 2600 3000
F 0 "R11" V 2600 3000 50  0000 C CNN
F 1 "R_0" V 2550 3175 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 3150 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3500 50  0001 C CNN
F 4 "DK" H 2600 3000 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3250 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3400 60  0001 C CNN "PurchasingLink"
	1    2600 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 3000 2950 3000
$Comp
L formula:R_100 R36
U 1 1 5DC27BC9
P 2600 3100
F 0 "R36" V 2600 3100 50  0000 C CNN
F 1 "R_0" V 2550 3275 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 3250 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3600 50  0001 C CNN
F 4 "DK" H 2600 3100 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3350 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3500 60  0001 C CNN "PurchasingLink"
	1    2600 3100
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 3100 2950 3100
$Comp
L formula:R_100 R13
U 1 1 5DC281FC
P 2600 3200
F 0 "R13" V 2600 3200 50  0000 C CNN
F 1 "R_0" V 2550 3375 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 3350 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3700 50  0001 C CNN
F 4 "DK" H 2600 3200 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3450 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3600 60  0001 C CNN "PurchasingLink"
	1    2600 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 3200 2950 3200
$Comp
L formula:R_100 R14
U 1 1 5DC2887F
P 2600 3300
F 0 "R14" V 2600 3300 50  0000 C CNN
F 1 "R_0" V 2550 3475 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 1800 3450 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 2300 3800 50  0001 C CNN
F 4 "DK" H 2600 3300 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 1950 3550 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 3080 3700 60  0001 C CNN "PurchasingLink"
	1    2600 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 3300 2950 3300
Wire Wire Line
	2450 2500 2600 2500
Wire Wire Line
	3050 2625 2600 2625
Wire Wire Line
	2600 2625 2600 2500
Connection ~ 2600 2500
Wire Wire Line
	2600 2500 2700 2500
Text Label 2950 1900 0    50   ~ 0
IO1
Text Label 2950 2000 0    50   ~ 0
IO2
Text Label 2950 2100 0    50   ~ 0
IO3
Text Label 2950 2200 0    50   ~ 0
IO4
Text Label 2950 2900 0    50   ~ 0
IO5
Text Label 2950 3000 0    50   ~ 0
IO6
Text Label 2950 3100 0    50   ~ 0
IO7
Text Label 2950 3200 0    50   ~ 0
IO8
Text Label 2950 3300 0    50   ~ 0
IO9
Text Label 10725 1975 0    50   ~ 0
IO4
Text Label 10725 1875 0    50   ~ 0
IO5
Text Label 10725 1775 0    50   ~ 0
IO6
Text Label 10725 1675 0    50   ~ 0
IO7
Text Label 10725 2275 0    50   ~ 0
IO1
Text Label 10725 2175 0    50   ~ 0
IO2
Text Label 10725 2075 0    50   ~ 0
IO3
Text Label 2950 2400 0    50   ~ 0
SCL
Text Label 2950 2300 0    50   ~ 0
SDA
Text Label 10725 1475 0    50   ~ 0
SDA
Text Label 10725 1575 0    50   ~ 0
SCL
Text Label 3675 2750 0    50   ~ 0
RX
Text Label 2950 2800 0    50   ~ 0
TX
Text Label 10725 1275 0    50   ~ 0
TX
Text Label 10725 1375 0    50   ~ 0
RX
Wire Wire Line
	2900 2300 2900 2225
Wire Wire Line
	2900 2225 3175 2225
Wire Wire Line
	3175 2225 3175 2075
Connection ~ 2900 2300
Wire Wire Line
	2900 2300 2950 2300
Wire Wire Line
	2900 2400 2900 2325
Wire Wire Line
	2900 2325 3250 2325
Wire Wire Line
	3250 2325 3250 2075
Connection ~ 2900 2400
Wire Wire Line
	2900 2400 2950 2400
$Comp
L formula:R_10K R16
U 1 1 5DC817DE
P 3175 1925
F 0 "R16" H 3075 2075 50  0000 L CNN
F 1 "R_10K" V 3175 1825 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 3105 1925 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 3255 1925 50  0001 C CNN
F 4 "DK" H 3175 1925 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 3175 1925 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 3655 2325 60  0001 C CNN "PurchasingLink"
	1    3175 1925
	1    0    0    -1  
$EndComp
$Comp
L formula:R_10K R17
U 1 1 5DC818A4
P 3250 1925
F 0 "R17" H 3300 2075 50  0000 L CNN
F 1 "R_10K" V 3250 1825 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 3180 1925 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 3330 1925 50  0001 C CNN
F 4 "DK" H 3250 1925 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 3250 1925 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 3730 2325 60  0001 C CNN "PurchasingLink"
	1    3250 1925
	1    0    0    -1  
$EndComp
$Comp
L formula:R_1K R18
U 1 1 5DC85874
P 3475 3000
F 0 "R18" V 3268 3000 50  0000 C CNN
F 1 "R_1K" V 3359 3000 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 3405 3000 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 3555 3000 50  0001 C CNN
F 4 "DK" H 3475 3000 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD1K00CT-ND" H 3475 3000 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD1K00CT-ND" H 3955 3400 60  0001 C CNN "PurchasingLink"
	1    3475 3000
	0    1    1    0   
$EndComp
$Comp
L formula:R_2.2K R19
U 1 1 5DC85B83
P 3900 3000
F 0 "R19" V 3693 3000 50  0000 C CNN
F 1 "R_2.2K" V 3784 3000 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 3830 3000 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 3980 3000 50  0001 C CNN
F 4 "DK" H 3900 3000 60  0001 C CNN "MFN"
F 5 "RMCF0805FT2K20CT-ND" H 3900 3000 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/stackpole-electronics-inc/RMCF0805FT2K20/RMCF0805FT2K20CT-ND/1942387" H 4380 3400 60  0001 C CNN "PurchasingLink"
	1    3900 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	3675 2750 3675 3000
Wire Wire Line
	3675 3000 3625 3000
Wire Wire Line
	3750 3000 3675 3000
Connection ~ 3675 3000
Wire Wire Line
	3325 3000 3325 2700
Wire Wire Line
	4050 3000 4175 3000
Wire Wire Line
	4175 3000 4175 3125
$Comp
L power:GND #PWR017
U 1 1 5DC89045
P 4175 3125
F 0 "#PWR017" H 4175 2875 50  0001 C CNN
F 1 "GND" H 4180 2952 50  0000 C CNN
F 2 "" H 4175 3125 50  0001 C CNN
F 3 "" H 4175 3125 50  0001 C CNN
	1    4175 3125
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2700 3325 2700
$Comp
L footprints:LED_0805 D6
U 1 1 5DD59D46
P 3200 1100
F 0 "D6" H 3300 1150 50  0000 C CNN
F 1 "LED_0805" H 3193 936 50  0001 C CNN
F 2 "footprints:LED_0805_OEM" H 3100 1100 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 3200 1200 50  0001 C CNN
F 4 "DK" H 3200 1100 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 3200 1100 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 3600 1600 60  0001 C CNN "PurchasingLink"
	1    3200 1100
	-1   0    0    1   
$EndComp
$Comp
L footprints:LED_0805 D8
U 1 1 5DD5D26D
P 3200 1250
F 0 "D8" H 3300 1300 50  0000 C CNN
F 1 "LED_0805" H 2750 1300 50  0001 C CNN
F 2 "footprints:LED_0805_OEM" H 3100 1250 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 3200 1350 50  0001 C CNN
F 4 "DK" H 3200 1250 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 3200 1250 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 3600 1750 60  0001 C CNN "PurchasingLink"
	1    3200 1250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2750 1100 3050 1100
Wire Wire Line
	3050 1250 2900 1250
Wire Wire Line
	2900 1250 2900 1200
Wire Wire Line
	2900 1200 2750 1200
$Comp
L footprints:R_200 R1
U 1 1 5DD6973C
P 2600 1100
F 0 "R1" V 2600 1100 50  0000 C CNN
F 1 "R_200" V 2550 1300 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 2530 1100 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 2680 1100 50  0001 C CNN
F 4 "DK" H 2600 1100 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 2600 1100 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 3080 1500 60  0001 C CNN "PurchasingLink"
	1    2600 1100
	0    1    1    0   
$EndComp
$Comp
L footprints:R_200 R31
U 1 1 5DD6AA08
P 2600 1200
F 0 "R31" V 2600 1200 50  0000 C CNN
F 1 "R_200" V 2550 1400 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 2530 1200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 2680 1200 50  0001 C CNN
F 4 "DK" H 2600 1200 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 2600 1200 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 3080 1600 60  0001 C CNN "PurchasingLink"
	1    2600 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	3650 1100 3350 1100
Wire Wire Line
	3650 1100 3650 1250
Wire Wire Line
	3650 1250 3350 1250
Wire Wire Line
	3650 1250 3650 1350
Connection ~ 3650 1250
$Comp
L power:GND #PWR016
U 1 1 5DD7661F
P 3650 1350
F 0 "#PWR016" H 3650 1100 50  0001 C CNN
F 1 "GND" H 3655 1177 50  0000 C CNN
F 2 "" H 3650 1350 50  0001 C CNN
F 3 "" H 3650 1350 50  0001 C CNN
	1    3650 1350
	1    0    0    -1  
$EndComp
Text Label 2450 1000 0    50   ~ 0
THROTTLE_SENSE
Text Label 2450 3400 0    50   ~ 0
STEERING_SENSE
Text Label 925  5950 2    50   ~ 0
MISO
Text Label 925  6050 2    50   ~ 0
SCK
Text Label 925  6150 2    50   ~ 0
RESET
Text Label 1425 6050 0    50   ~ 0
MOSI
$Comp
L power:GND #PWR09
U 1 1 5DC1B69A
P 1425 6150
F 0 "#PWR09" H 1425 5900 50  0001 C CNN
F 1 "GND" H 1430 5977 50  0000 C CNN
F 2 "" H 1425 6150 50  0001 C CNN
F 3 "" H 1425 6150 50  0001 C CNN
	1    1425 6150
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J4
U 1 1 5DC7F6F5
P 10525 4775
F 0 "J4" H 10553 4816 50  0000 L CNN
F 1 "CONN_01X03" H 10553 4725 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10525 3575 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10525 3575 50  0001 C CNN
F 4 "Mouser" H 10525 4775 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10525 4775 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10925 5375 60  0001 C CNN "PurchasingLink"
	1    10525 4775
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J5
U 1 1 5DC8480A
P 10525 5100
F 0 "J5" H 10553 5141 50  0000 L CNN
F 1 "CONN_01X03" H 10553 5050 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10525 3900 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10525 3900 50  0001 C CNN
F 4 "Mouser" H 10525 5100 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10525 5100 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10925 5700 60  0001 C CNN "PurchasingLink"
	1    10525 5100
	1    0    0    -1  
$EndComp
Text Label 10275 4775 2    50   ~ 0
TX2
Text Label 10275 4875 2    50   ~ 0
RX2
$Comp
L power:GND #PWR031
U 1 1 5DC893F7
P 10275 5000
F 0 "#PWR031" H 10275 4750 50  0001 C CNN
F 1 "GND" V 10280 4872 50  0000 R CNN
F 2 "" H 10275 5000 50  0001 C CNN
F 3 "" H 10275 5000 50  0001 C CNN
	1    10275 5000
	0    1    1    0   
$EndComp
$Comp
L formula:CONN_01X03 J6
U 1 1 5DC89E84
P 10550 5625
F 0 "J6" H 10578 5666 50  0000 L CNN
F 1 "CONN_01X03" H 10578 5575 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10550 4425 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10550 4425 50  0001 C CNN
F 4 "Mouser" H 10550 5625 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10550 5625 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10950 6225 60  0001 C CNN "PurchasingLink"
	1    10550 5625
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J7
U 1 1 5DC89E8E
P 10550 5950
F 0 "J7" H 10578 5991 50  0000 L CNN
F 1 "CONN_01X03" H 10578 5900 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10550 4750 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10550 4750 50  0001 C CNN
F 4 "Mouser" H 10550 5950 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10550 5950 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10950 6550 60  0001 C CNN "PurchasingLink"
	1    10550 5950
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J8
U 1 1 5DC8AD06
P 10550 6275
F 0 "J8" H 10578 6316 50  0000 L CNN
F 1 "CONN_01X03" H 10578 6225 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10550 5075 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10550 5075 50  0001 C CNN
F 4 "Mouser" H 10550 6275 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10550 6275 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10950 6875 60  0001 C CNN "PurchasingLink"
	1    10550 6275
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR035
U 1 1 5DC8BF81
P 10300 5725
F 0 "#PWR035" H 10300 5475 50  0001 C CNN
F 1 "GND" V 10305 5597 50  0000 R CNN
F 2 "" H 10300 5725 50  0001 C CNN
F 3 "" H 10300 5725 50  0001 C CNN
	1    10300 5725
	0    1    1    0   
$EndComp
Text Label 10300 5950 2    50   ~ 0
SDA
Text Label 10300 5850 2    50   ~ 0
SCL
NoConn ~ 10300 6050
NoConn ~ 10300 6175
NoConn ~ 10300 6275
NoConn ~ 10300 6375
NoConn ~ 10275 4675
NoConn ~ 10275 5200
$Comp
L formula:Crystal_SMD Y2
U 1 1 5DDC1874
P 5300 4475
F 0 "Y2" H 5250 4775 50  0000 L CNN
F 1 "Crystal_SMD" H 5100 4675 50  0000 L CNN
F 2 "footprints:Crystal_SMD_FA238" H 5250 4550 50  0001 C CNN
F 3 "http://www.txccorp.com/download/products/quartz_crystals/2015TXC_7M_17.pdf" H 5350 4650 50  0001 C CNN
F 4 "DK" H 5300 4475 60  0001 C CNN "MFN"
F 5 "887-1125-1-ND" H 5300 4475 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/txc-corporation/7M-16.000MAAJ-T/887-1125-1-ND/2119014" H 5750 5050 60  0001 C CNN "PurchasingLink"
	1    5300 4475
	1    0    0    -1  
$EndComp
$Comp
L formula:C_0.1uF C4
U 1 1 5DDC187D
P 4800 2650
F 0 "C4" V 4750 2500 50  0000 L CNN
F 1 "C_0.1uF" V 4750 2700 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 4838 2500 50  0001 C CNN
F 3 "http://datasheets.avx.com/X7RDielectric.pdf" H 4825 2750 50  0001 C CNN
F 4 "DK" H 4800 2650 60  0001 C CNN "MFN"
F 5 "478-3352-1-ND" H 4800 2650 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=478-3352-1-ND" H 5225 3150 60  0001 C CNN "PurchasingLink"
	1    4800 2650
	1    0    0    -1  
$EndComp
$Comp
L formula:C_30pF C9
U 1 1 5DDC1886
P 5100 4775
F 0 "C9" V 5050 4625 50  0000 L CNN
F 1 "C_30pF" V 5050 4825 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 5138 4625 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Samsung%20PDFs/CL_Series_MLCC_ds.pdf" H 5125 4875 50  0001 C CNN
F 4 "DK" H 5100 4775 60  0001 C CNN "MFN"
F 5 "1276-1130-1-ND" H 5100 4775 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics-america-inc/CL21C300JBANNNC/1276-1130-1-ND/3889216" H 5525 5275 60  0001 C CNN "PurchasingLink"
	1    5100 4775
	1    0    0    -1  
$EndComp
$Comp
L formula:C_30pF C6
U 1 1 5DDC188F
P 5500 4775
F 0 "C6" V 5450 4625 50  0000 L CNN
F 1 "C_30pF" V 5450 4825 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 5538 4625 50  0001 C CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Samsung%20PDFs/CL_Series_MLCC_ds.pdf" H 5525 4875 50  0001 C CNN
F 4 "DK" H 5500 4775 60  0001 C CNN "MFN"
F 5 "1276-1130-1-ND" H 5500 4775 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics-america-inc/CL21C300JBANNNC/1276-1130-1-ND/3889216" H 5925 5275 60  0001 C CNN "PurchasingLink"
	1    5500 4775
	1    0    0    -1  
$EndComp
$Comp
L formula:R_10K R26
U 1 1 5DDC1898
P 6900 2700
F 0 "R26" V 6900 2700 50  0000 C CNN
F 1 "R_10K" V 6850 2950 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 6830 2700 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 6980 2700 50  0001 C CNN
F 4 "DK" H 6900 2700 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 6900 2700 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 7380 3100 60  0001 C CNN "PurchasingLink"
	1    6900 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 4475 5500 4475
Wire Wire Line
	5500 4475 5500 4525
Wire Wire Line
	5200 4475 5100 4475
Wire Wire Line
	5100 4475 5100 4525
$Comp
L power:GND #PWR020
U 1 1 5DDC18A2
P 5100 4925
F 0 "#PWR020" H 5100 4675 50  0001 C CNN
F 1 "GND" H 5105 4752 50  0000 C CNN
F 2 "" H 5100 4925 50  0001 C CNN
F 3 "" H 5100 4925 50  0001 C CNN
	1    5100 4925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5DDC18A8
P 5500 4925
F 0 "#PWR022" H 5500 4675 50  0001 C CNN
F 1 "GND" H 5505 4752 50  0000 C CNN
F 2 "" H 5500 4925 50  0001 C CNN
F 3 "" H 5500 4925 50  0001 C CNN
	1    5500 4925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5DDC18AE
P 5300 4775
F 0 "#PWR021" H 5300 4525 50  0001 C CNN
F 1 "GND" H 5305 4602 50  0000 C CNN
F 2 "" H 5300 4775 50  0001 C CNN
F 3 "" H 5300 4775 50  0001 C CNN
	1    5300 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4625 5300 4775
$Comp
L power:GND #PWR023
U 1 1 5DDC18B5
P 5600 4375
F 0 "#PWR023" H 5600 4125 50  0001 C CNN
F 1 "GND" H 5700 4375 50  0000 C CNN
F 2 "" H 5600 4375 50  0001 C CNN
F 3 "" H 5600 4375 50  0001 C CNN
	1    5600 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 4325 5600 4375
Wire Wire Line
	5300 4325 5600 4325
Text Label 6500 1800 0    50   ~ 0
XTAL3
Text Label 6500 1900 0    50   ~ 0
XTAL4
Text Label 4900 4525 2    50   ~ 0
XTAL3
Text Label 5800 4525 0    50   ~ 0
XTAL4
Wire Wire Line
	4900 4525 5100 4525
Connection ~ 5100 4525
Wire Wire Line
	5100 4525 5100 4625
Wire Wire Line
	5800 4525 5500 4525
Connection ~ 5500 4525
Wire Wire Line
	5500 4525 5500 4625
Text Label 7100 2825 0    50   ~ 0
RESET2
Wire Wire Line
	7050 2700 7300 2700
Text Label 6500 1500 0    50   ~ 0
MOSI2
Text Label 6500 1600 0    50   ~ 0
MISO2
Text Label 6500 1700 0    50   ~ 0
SCK2
Wire Wire Line
	4800 2500 4800 1500
Wire Wire Line
	4800 1200 4900 1200
Wire Wire Line
	4800 2800 4800 3300
Wire Wire Line
	4800 3300 4900 3300
Wire Wire Line
	4800 3300 4800 3900
Connection ~ 4800 3300
Wire Wire Line
	4900 1500 4800 1500
Connection ~ 4800 1500
Wire Wire Line
	4800 1500 4800 1200
$Comp
L power:GND #PWR019
U 1 1 5DDC18D5
P 4800 3900
F 0 "#PWR019" H 4800 3650 50  0001 C CNN
F 1 "GND" H 4805 3727 50  0000 C CNN
F 2 "" H 4800 3900 50  0001 C CNN
F 3 "" H 4800 3900 50  0001 C CNN
	1    4800 3900
	1    0    0    -1  
$EndComp
NoConn ~ 4900 1950
$Comp
L formula:CONN_02X03 J3
U 1 1 5DDC18DF
P 6875 4750
F 0 "J3" H 6675 5150 50  0000 C CNN
F 1 "CONN_02X03" H 6675 5050 50  0000 C CNN
F 2 "footprints:Pin_Header_Straight_2x03" H 6875 3550 50  0001 C CNN
F 3 "http://portal.fciconnect.com/Comergent//fci/drawing/67996.pdf" H 6875 3550 50  0001 C CNN
F 4 "DK" H 6875 4750 60  0001 C CNN "MFN"
F 5 "609-3234-ND" H 6875 4750 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/amphenol-fci/67997-206HLF/609-3234-ND/1878491" H 7275 5350 60  0001 C CNN "PurchasingLink"
	1    6875 4750
	1    0    0    -1  
$EndComp
Text Label 6625 4650 2    50   ~ 0
MISO2
Text Label 6625 4750 2    50   ~ 0
SCK2
Text Label 6625 4850 2    50   ~ 0
RESET2
Text Label 7125 4750 0    50   ~ 0
MOSI2
$Comp
L power:GND #PWR025
U 1 1 5DDC18E9
P 7125 4850
F 0 "#PWR025" H 7125 4600 50  0001 C CNN
F 1 "GND" H 7130 4677 50  0000 C CNN
F 2 "" H 7125 4850 50  0001 C CNN
F 3 "" H 7125 4850 50  0001 C CNN
	1    7125 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1000 4800 1200
Connection ~ 4800 1200
NoConn ~ 4900 2150
NoConn ~ 4900 2250
Wire Wire Line
	6500 2700 6650 2700
Wire Wire Line
	7100 2825 6650 2825
Wire Wire Line
	6650 2825 6650 2700
Connection ~ 6650 2700
Wire Wire Line
	6650 2700 6750 2700
NoConn ~ 6500 3600
Text Label 7000 2600 0    50   ~ 0
SCL
Text Label 7000 2500 0    50   ~ 0
SDA
Wire Wire Line
	6950 2500 6950 2425
Wire Wire Line
	6950 2425 7225 2425
Wire Wire Line
	7225 2425 7225 2275
Connection ~ 6950 2500
Wire Wire Line
	6950 2500 7000 2500
Wire Wire Line
	6950 2600 6950 2525
Wire Wire Line
	6950 2525 7300 2525
Wire Wire Line
	7300 2525 7300 2275
Connection ~ 6950 2600
Wire Wire Line
	6950 2600 7000 2600
$Comp
L formula:R_10K R27
U 1 1 5DDC199D
P 7225 2125
F 0 "R27" H 7125 2275 50  0000 L CNN
F 1 "R_10K" V 7225 2025 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 7155 2125 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 7305 2125 50  0001 C CNN
F 4 "DK" H 7225 2125 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 7225 2125 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 7705 2525 60  0001 C CNN "PurchasingLink"
	1    7225 2125
	1    0    0    -1  
$EndComp
$Comp
L formula:R_10K R28
U 1 1 5DDC19A6
P 7300 2125
F 0 "R28" H 7350 2275 50  0000 L CNN
F 1 "R_10K" V 7300 2025 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 7230 2125 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 7380 2125 50  0001 C CNN
F 4 "DK" H 7300 2125 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 7300 2125 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 7780 2525 60  0001 C CNN "PurchasingLink"
	1    7300 2125
	1    0    0    -1  
$EndComp
$Comp
L footprints:LED_0805 D4
U 1 1 5DDC19DB
P 7250 1300
F 0 "D4" H 7350 1350 50  0000 C CNN
F 1 "LED_0805" H 7243 1136 50  0001 C CNN
F 2 "footprints:LED_0805_OEM" H 7150 1300 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 7250 1400 50  0001 C CNN
F 4 "DK" H 7250 1300 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 7250 1300 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 7650 1800 60  0001 C CNN "PurchasingLink"
	1    7250 1300
	-1   0    0    1   
$EndComp
$Comp
L footprints:LED_0805 D5
U 1 1 5DDC19E4
P 7250 1450
F 0 "D5" H 7350 1500 50  0000 C CNN
F 1 "LED_0805" H 6800 1500 50  0001 C CNN
F 2 "footprints:LED_0805_OEM" H 7150 1450 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 7250 1550 50  0001 C CNN
F 4 "DK" H 7250 1450 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 7250 1450 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 7650 1950 60  0001 C CNN "PurchasingLink"
	1    7250 1450
	-1   0    0    1   
$EndComp
$Comp
L footprints:LED_0805 D3
U 1 1 5DDC19ED
P 7250 1150
F 0 "D3" H 7350 1200 50  0000 C CNN
F 1 "LED_0805" H 7243 986 50  0001 C CNN
F 2 "footprints:LED_0805_OEM" H 7150 1150 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 7250 1250 50  0001 C CNN
F 4 "DK" H 7250 1150 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 7250 1150 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 7650 1650 60  0001 C CNN "PurchasingLink"
	1    7250 1150
	-1   0    0    1   
$EndComp
Wire Wire Line
	6800 1300 7100 1300
Wire Wire Line
	7100 1150 6950 1150
Wire Wire Line
	6950 1150 6950 1200
Wire Wire Line
	6950 1200 6800 1200
Wire Wire Line
	7100 1450 6950 1450
Wire Wire Line
	6950 1450 6950 1400
Wire Wire Line
	6950 1400 6800 1400
$Comp
L footprints:R_200 R20
U 1 1 5DDC19FD
P 6650 1200
F 0 "R20" V 6650 1200 50  0000 C CNN
F 1 "R_200" V 6600 1400 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 6580 1200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 6730 1200 50  0001 C CNN
F 4 "DK" H 6650 1200 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 6650 1200 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 7130 1600 60  0001 C CNN "PurchasingLink"
	1    6650 1200
	0    1    1    0   
$EndComp
$Comp
L footprints:R_200 R21
U 1 1 5DDC1A06
P 6650 1300
F 0 "R21" V 6650 1300 50  0000 C CNN
F 1 "R_200" V 6600 1500 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 6580 1300 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 6730 1300 50  0001 C CNN
F 4 "DK" H 6650 1300 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 6650 1300 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 7130 1700 60  0001 C CNN "PurchasingLink"
	1    6650 1300
	0    1    1    0   
$EndComp
$Comp
L footprints:R_200 R22
U 1 1 5DDC1A0F
P 6650 1400
F 0 "R22" V 6650 1400 50  0000 C CNN
F 1 "R_200" V 6600 1600 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 6580 1400 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 6730 1400 50  0001 C CNN
F 4 "DK" H 6650 1400 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 6650 1400 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 7130 1800 60  0001 C CNN "PurchasingLink"
	1    6650 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 1150 7700 1150
Wire Wire Line
	7700 1150 7700 1300
Wire Wire Line
	7700 1300 7400 1300
Wire Wire Line
	7700 1300 7700 1450
Wire Wire Line
	7700 1450 7400 1450
Connection ~ 7700 1300
Wire Wire Line
	7700 1450 7700 1550
Connection ~ 7700 1450
$Comp
L power:GND #PWR029
U 1 1 5DDC1A1D
P 7700 1550
F 0 "#PWR029" H 7700 1300 50  0001 C CNN
F 1 "GND" H 7705 1377 50  0000 C CNN
F 2 "" H 7700 1550 50  0001 C CNN
F 3 "" H 7700 1550 50  0001 C CNN
	1    7700 1550
	1    0    0    -1  
$EndComp
NoConn ~ 6500 2100
NoConn ~ 6500 2200
NoConn ~ 6500 2300
NoConn ~ 6500 2400
NoConn ~ 6500 3100
NoConn ~ 6500 3200
NoConn ~ 6500 3300
NoConn ~ 6500 3400
NoConn ~ 6500 3500
Text Label 7725 2950 0    50   ~ 0
RX2
$Comp
L formula:R_1K R29
U 1 1 5DEC60BE
P 7525 3200
F 0 "R29" V 7318 3200 50  0000 C CNN
F 1 "R_1K" V 7409 3200 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 7455 3200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 7605 3200 50  0001 C CNN
F 4 "DK" H 7525 3200 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD1K00CT-ND" H 7525 3200 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD1K00CT-ND" H 8005 3600 60  0001 C CNN "PurchasingLink"
	1    7525 3200
	0    1    1    0   
$EndComp
$Comp
L formula:R_2.2K R30
U 1 1 5DEC60C7
P 7950 3200
F 0 "R30" V 7743 3200 50  0000 C CNN
F 1 "R_2.2K" V 7834 3200 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 7880 3200 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 8030 3200 50  0001 C CNN
F 4 "DK" H 7950 3200 60  0001 C CNN "MFN"
F 5 "RMCF0805FT2K20CT-ND" H 7950 3200 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/stackpole-electronics-inc/RMCF0805FT2K20/RMCF0805FT2K20CT-ND/1942387" H 8430 3600 60  0001 C CNN "PurchasingLink"
	1    7950 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	7725 2950 7725 3200
Wire Wire Line
	7725 3200 7675 3200
Wire Wire Line
	7800 3200 7725 3200
Connection ~ 7725 3200
Wire Wire Line
	7375 3200 7375 2900
Wire Wire Line
	8100 3200 8225 3200
Wire Wire Line
	8225 3200 8225 3325
$Comp
L power:GND #PWR030
U 1 1 5DEC60D4
P 8225 3325
F 0 "#PWR030" H 8225 3075 50  0001 C CNN
F 1 "GND" H 8230 3152 50  0000 C CNN
F 2 "" H 8225 3325 50  0001 C CNN
F 3 "" H 8225 3325 50  0001 C CNN
	1    8225 3325
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2900 7375 2900
$Comp
L formula:R_100 R25
U 1 1 5DECC6E8
P 6650 3000
F 0 "R25" V 6650 3000 50  0000 C CNN
F 1 "R_0" V 6600 3175 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 5850 3150 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 6350 3500 50  0001 C CNN
F 4 "DK" H 6650 3000 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD100RCT-ND" H 6000 3250 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD100RCT-ND" H 7130 3400 60  0001 C CNN "PurchasingLink"
	1    6650 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 3000 7000 3000
Text Label 7000 3000 0    50   ~ 0
TX2
$Comp
L formula:C_0.1uF C1
U 1 1 5DB864DC
P 750 2450
F 0 "C1" V 700 2300 50  0000 L CNN
F 1 "C_0.1uF" V 700 2500 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 788 2300 50  0001 C CNN
F 3 "http://datasheets.avx.com/X7RDielectric.pdf" H 775 2550 50  0001 C CNN
F 4 "DK" H 750 2450 60  0001 C CNN "MFN"
F 5 "478-3352-1-ND" H 750 2450 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=478-3352-1-ND" H 1175 2950 60  0001 C CNN "PurchasingLink"
	1    750  2450
	1    0    0    -1  
$EndComp
$Comp
L footprints:ATmega328P-AU U1
U 1 1 5DF00453
P 1850 2200
F 0 "U1" H 1650 3767 50  0000 C CNN
F 1 "ATmega328P-AU" H 1650 3676 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 1650 3750 50  0001 C CIN
F 3 "" H 1850 2200 50  0001 C CNN
	1    1850 2200
	1    0    0    -1  
$EndComp
$Comp
L footprints:ATmega328P-AU U3
U 1 1 5DF02518
P 5900 2400
F 0 "U3" H 5700 3967 50  0000 C CNN
F 1 "ATmega328P-AU" H 5700 3876 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 5700 3950 50  0001 C CIN
F 3 "" H 5900 2400 50  0001 C CNN
	1    5900 2400
	1    0    0    -1  
$EndComp
NoConn ~ 4900 1300
NoConn ~ 4900 3400
NoConn ~ 4900 3500
NoConn ~ 850  3200
NoConn ~ 850  3300
NoConn ~ 850  1100
$Comp
L formula:CONN_02X03 J1
U 1 1 5DF1E4E1
P 1175 6050
F 0 "J1" H 1175 6365 50  0000 C CNN
F 1 "CONN_02X03" H 1175 6274 50  0000 C CNN
F 2 "footprints:Pin_Header_Straight_2x03" H 1175 4850 50  0001 C CNN
F 3 "http://portal.fciconnect.com/Comergent//fci/drawing/67996.pdf" H 1175 4850 50  0001 C CNN
F 4 "DK" H 1175 6050 60  0001 C CNN "MFN"
F 5 "609-3234-ND" H 1175 6050 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/amphenol-fci/67997-206HLF/609-3234-ND/1878491" H 1575 6650 60  0001 C CNN "PurchasingLink"
	1    1175 6050
	1    0    0    -1  
$EndComp
Text Label 10725 1175 0    50   ~ 0
IO8
Text Label 10725 1075 0    50   ~ 0
IO9
NoConn ~ 10725 775 
NoConn ~ 10725 875 
NoConn ~ 10725 975 
$Comp
L formula:CONN_01X03 J9
U 1 1 5DF44445
P 9000 4800
F 0 "J9" H 9028 4841 50  0000 L CNN
F 1 "CONN_01X03" H 9028 4750 50  0000 L CNN
F 2 "footprints:MHSS1105" H 9000 3600 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 9000 3600 50  0001 C CNN
F 4 "Mouser" H 9000 4800 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 9000 4800 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 9400 5400 60  0001 C CNN "PurchasingLink"
	1    9000 4800
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J10
U 1 1 5DF4784D
P 9000 5525
F 0 "J10" H 9028 5566 50  0000 L CNN
F 1 "CONN_01X03" H 9028 5475 50  0000 L CNN
F 2 "footprints:MHSS1105" H 9000 4325 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 9000 4325 50  0001 C CNN
F 4 "Mouser" H 9000 5525 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 9000 5525 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 9400 6125 60  0001 C CNN "PurchasingLink"
	1    9000 5525
	1    0    0    -1  
$EndComp
Text Label 8750 4800 2    50   ~ 0
THROTTLE_SENSE
Text Label 8750 5525 2    50   ~ 0
STEERING_SENSE
$Comp
L power:VCC #PWR03
U 1 1 5DF51117
P 875 7175
F 0 "#PWR03" H 875 7025 50  0001 C CNN
F 1 "VCC" H 892 7348 50  0000 C CNN
F 2 "" H 875 7175 50  0001 C CNN
F 3 "" H 875 7175 50  0001 C CNN
	1    875  7175
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR01
U 1 1 5DF54886
P 750 800
F 0 "#PWR01" H 750 650 50  0001 C CNN
F 1 "VCC" H 767 973 50  0000 C CNN
F 2 "" H 750 800 50  0001 C CNN
F 3 "" H 750 800 50  0001 C CNN
	1    750  800 
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR011
U 1 1 5DF57FC0
P 4800 1000
F 0 "#PWR011" H 4800 850 50  0001 C CNN
F 1 "VCC" H 4817 1173 50  0000 C CNN
F 2 "" H 4800 1000 50  0001 C CNN
F 3 "" H 4800 1000 50  0001 C CNN
	1    4800 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 5DF587EF
P 8750 4900
F 0 "#PWR036" H 8750 4650 50  0001 C CNN
F 1 "GND" H 8755 4727 50  0000 C CNN
F 2 "" H 8750 4900 50  0001 C CNN
F 3 "" H 8750 4900 50  0001 C CNN
	1    8750 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 5DF593D3
P 8750 5625
F 0 "#PWR038" H 8750 5375 50  0001 C CNN
F 1 "GND" H 8755 5452 50  0000 C CNN
F 2 "" H 8750 5625 50  0001 C CNN
F 3 "" H 8750 5625 50  0001 C CNN
	1    8750 5625
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR018
U 1 1 5DF62DDF
P 8750 4700
F 0 "#PWR018" H 8750 4550 50  0001 C CNN
F 1 "VCC" H 8767 4873 50  0000 C CNN
F 2 "" H 8750 4700 50  0001 C CNN
F 3 "" H 8750 4700 50  0001 C CNN
	1    8750 4700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR037
U 1 1 5DF63B82
P 8750 5425
F 0 "#PWR037" H 8750 5275 50  0001 C CNN
F 1 "VCC" H 8767 5598 50  0000 C CNN
F 2 "" H 8750 5425 50  0001 C CNN
F 3 "" H 8750 5425 50  0001 C CNN
	1    8750 5425
	1    0    0    -1  
$EndComp
$Comp
L footprints:TPS561201 U2
U 1 1 5DF6B0F4
P 4025 6175
F 0 "U2" H 4025 6662 60  0000 C CNN
F 1 "TPS561201" H 4025 6556 60  0000 C CNN
F 2 "footprints:SOT-23-6_OEM" H 3725 6425 60  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps561201.pdf" H 3825 6525 60  0001 C CNN
F 4 "DK" H 4025 6175 60  0001 C CNN "MFN"
F 5 "TPS561201" H 4025 6175 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=296-46928-1-ND" H 4225 6925 60  0001 C CNN "PurchasingLink"
	1    4025 6175
	1    0    0    -1  
$EndComp
$Comp
L footprints:D_Zener_18V D7
U 1 1 5DF6FA61
P 2650 6450
F 0 "D7" V 2604 6529 50  0000 L CNN
F 1 "D_Zener_18V" V 2695 6529 50  0000 L CNN
F 2 "footprints:DO-214AA" H 2550 6450 50  0001 C CNN
F 3 "http://www.mccsemi.com/up_pdf/SMBJ5338B-SMBJ5388B(SMB).pdf" H 2650 6550 50  0001 C CNN
F 4 "DK" H 2850 6750 60  0001 C CNN "MFN"
F 5 "SMBJ5355B-TPMSCT-ND" H 2750 6650 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=SMBJ5355B-TPMSCT-ND" H 3050 6950 60  0001 C CNN "PurchasingLink"
	1    2650 6450
	0    1    1    0   
$EndComp
$Comp
L footprints:F_500mA_16V F1
U 1 1 5DF71CF0
P 2950 6150
F 0 "F1" H 3010 6196 50  0000 L CNN
F 1 "F_500mA_16V" H 3010 6105 50  0000 L CNN
F 2 "footprints:Fuse_1210" V 2880 6150 50  0001 C CNN
F 3 "https://belfuse.com/resources/CircuitProtection/datasheets/0ZCH%20Nov2016.pdf" V 3030 6150 50  0001 C CNN
F 4 "DK" H 2950 6150 60  0001 C CNN "MFN"
F 5 "507-1786-1-ND" H 2950 6150 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/bel-fuse-inc/0ZCH0050FF2G/507-1786-1-ND/4156209" V 3430 6550 60  0001 C CNN "PurchasingLink"
	1    2950 6150
	1    0    0    -1  
$EndComp
$Comp
L footprints:R_1K R2
U 1 1 5DF72D1C
P 2950 6675
F 0 "R2" H 3020 6721 50  0000 L CNN
F 1 "R_1K" H 3020 6630 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 2880 6675 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-rncp.pdf" H 3030 6675 50  0001 C CNN
F 4 "DK" H 2950 6675 60  0001 C CNN "MFN"
F 5 "RNCP0805FTD1K00CT-ND" H 2950 6675 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RNCP0805FTD1K00CT-ND" H 3430 7075 60  0001 C CNN "PurchasingLink"
	1    2950 6675
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 6300 2950 6350
$Comp
L footprints:LED_0805 D1
U 1 1 5DF78713
P 2950 7050
F 0 "D1" V 2989 6933 50  0000 R CNN
F 1 "LED_0805" V 2898 6933 50  0000 R CNN
F 2 "footprints:LED_0805_OEM" H 2850 7050 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 2950 7150 50  0001 C CNN
F 4 "DK" H 2950 7050 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 2950 7050 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 3350 7550 60  0001 C CNN "PurchasingLink"
	1    2950 7050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2950 6825 2950 6900
Wire Wire Line
	2650 6300 2650 5925
Wire Wire Line
	2650 5925 2950 5925
Wire Wire Line
	2950 5925 2950 6000
Wire Wire Line
	2950 5925 2950 5775
Connection ~ 2950 5925
$Comp
L power:GND #PWR039
U 1 1 5DF857D5
P 2650 6600
F 0 "#PWR039" H 2650 6350 50  0001 C CNN
F 1 "GND" H 2655 6427 50  0000 C CNN
F 2 "" H 2650 6600 50  0001 C CNN
F 3 "" H 2650 6600 50  0001 C CNN
	1    2650 6600
	1    0    0    -1  
$EndComp
$Comp
L footprints:R_10K R3
U 1 1 5DF869F8
P 3425 6275
F 0 "R3" V 3218 6275 50  0000 C CNN
F 1 "R_10K" V 3309 6275 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 3355 6275 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 3505 6275 50  0001 C CNN
F 4 "DK" H 3425 6275 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 3425 6275 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 3905 6675 60  0001 C CNN "PurchasingLink"
	1    3425 6275
	0    1    1    0   
$EndComp
Wire Wire Line
	3275 6275 3150 6275
Wire Wire Line
	3150 6275 3150 6350
Wire Wire Line
	3150 6350 2950 6350
Connection ~ 2950 6350
Wire Wire Line
	2950 6350 2950 6525
Wire Wire Line
	3150 6275 3150 5975
Wire Wire Line
	3150 5975 3575 5975
Connection ~ 3150 6275
Wire Wire Line
	3150 6350 3150 6425
Wire Wire Line
	3150 6425 3325 6425
Wire Wire Line
	3325 6425 3325 6500
Connection ~ 3150 6350
$Comp
L footprints:C_22uF C3
U 1 1 5DF93DE4
P 3325 6650
F 0 "C3" H 3440 6696 50  0000 L CNN
F 1 "C_22uF" H 3440 6605 50  0000 L CNN
F 2 "footprints:C_1206_OEM" H 3363 6500 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_general_en.pdf" H 3350 6750 50  0001 C CNN
F 4 "DK" H 3325 6650 60  0001 C CNN "MFN"
F 5 "445-11693-1-ND" H 3325 6650 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/tdk-corporation/C3216JB1C226M160AB/445-11693-1-ND/3953359" H 3750 7150 60  0001 C CNN "PurchasingLink"
	1    3325 6650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR042
U 1 1 5DF94CAC
P 3325 6800
F 0 "#PWR042" H 3325 6550 50  0001 C CNN
F 1 "GND" H 3330 6627 50  0000 C CNN
F 2 "" H 3325 6800 50  0001 C CNN
F 3 "" H 3325 6800 50  0001 C CNN
	1    3325 6800
	1    0    0    -1  
$EndComp
$Comp
L footprints:R_10K R4
U 1 1 5DF95F9A
P 4475 6425
F 0 "R4" H 4545 6471 50  0000 L CNN
F 1 "R_10K" H 4545 6380 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 4405 6425 50  0001 C CNN
F 3 "http://www.bourns.com/data/global/pdfs/CRS.pdf" H 4555 6425 50  0001 C CNN
F 4 "DK" H 4475 6425 60  0001 C CNN "MFN"
F 5 "CRS0805-FX-1002ELFCT-ND" H 4475 6425 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=CRS0805-FX-1002ELFCT-ND" H 4955 6825 60  0001 C CNN "PurchasingLink"
	1    4475 6425
	1    0    0    -1  
$EndComp
Wire Wire Line
	4475 6275 4625 6275
Connection ~ 4475 6275
$Comp
L footprints:R_51.1K R5
U 1 1 5DF9B5F5
P 4775 6275
F 0 "R5" V 4568 6275 50  0000 C CNN
F 1 "R_51.1K" V 4659 6275 50  0000 C CNN
F 2 "footprints:R_0805_OEM" H 3675 6575 50  0001 L CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDA0000/AOA0000C304.pdf" H 3675 6675 50  0001 L CNN
F 4 "DK" H 4775 6275 60  0001 C CNN "MFN"
F 5 "P51.1KCCT-ND" H 3675 6475 60  0001 L CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/panasonic-electronic-components/ERJ-6ENF5112V/P51.1KCCT-ND/119466" H 3675 6775 60  0001 L CNN "PurchasingLink"
	1    4775 6275
	0    1    1    0   
$EndComp
$Comp
L footprints:C_0.1uF C5
U 1 1 5DF9C889
P 4625 5975
F 0 "C5" V 4373 5975 50  0000 C CNN
F 1 "C_0.1uF" V 4464 5975 50  0000 C CNN
F 2 "footprints:C_0805_OEM" H 4663 5825 50  0001 C CNN
F 3 "http://datasheets.avx.com/X7RDielectric.pdf" H 4650 6075 50  0001 C CNN
F 4 "DK" H 4625 5975 60  0001 C CNN "MFN"
F 5 "478-3352-1-ND" H 4625 5975 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=478-3352-1-ND" H 5050 6475 60  0001 C CNN "PurchasingLink"
	1    4625 5975
	0    1    1    0   
$EndComp
$Comp
L footprints:L_4.7uH L1
U 1 1 5DF9DDAD
P 5000 6075
F 0 "L1" V 5185 6075 50  0000 C CNN
F 1 "L_4.7uH" V 5094 6075 50  0000 C CNN
F 2 "footprints:4.7uH_Inductor_OEM" H 4930 6015 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/documents/wdcatalog/withdrawn_inductor_commercial_power_vlp8040_en.pdf" H 5030 6115 50  0001 C CNN
F 4 "DK" H 5000 6075 60  0001 C CNN "MFN"
F 5 "445-6583-1-ND" H 5000 6075 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/tdk-corporation/VLP8040T-4R7M/445-6583-1-ND/2465893" H 5430 6515 60  0001 C CNN "PurchasingLink"
	1    5000 6075
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4775 5975 4775 6075
Wire Wire Line
	4475 6075 4775 6075
Connection ~ 4775 6075
Wire Wire Line
	4775 6075 4900 6075
$Comp
L power:GND #PWR043
U 1 1 5DFA8227
P 4025 6625
F 0 "#PWR043" H 4025 6375 50  0001 C CNN
F 1 "GND" H 4030 6452 50  0000 C CNN
F 2 "" H 4025 6625 50  0001 C CNN
F 3 "" H 4025 6625 50  0001 C CNN
	1    4025 6625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5DFA90B8
P 4475 6575
F 0 "#PWR044" H 4475 6325 50  0001 C CNN
F 1 "GND" H 4480 6402 50  0000 C CNN
F 2 "" H 4475 6575 50  0001 C CNN
F 3 "" H 4475 6575 50  0001 C CNN
	1    4475 6575
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 6075 5300 6075
Wire Wire Line
	5300 6075 5300 6275
Wire Wire Line
	5300 6275 5175 6275
$Comp
L footprints:C_47uF C12
U 1 1 5DFAF39B
P 5175 6425
F 0 "C12" H 5290 6471 50  0000 L CNN
F 1 "C_47uF" H 5290 6380 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 5213 6275 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2018/06/22/CL21A476MQYNNNG.pdf" H 5200 6525 50  0001 C CNN
F 4 "DK" H 5175 6425 60  0001 C CNN "MFN"
F 5 "1276-6467-1-ND" H 5175 6425 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL21A476MQYNNNG/1276-6467-1-ND/5958095" H 5600 6925 60  0001 C CNN "PurchasingLink"
	1    5175 6425
	1    0    0    -1  
$EndComp
Connection ~ 5175 6275
Wire Wire Line
	5175 6275 4925 6275
$Comp
L footprints:C_1uF C11
U 1 1 5DFB0409
P 5450 6425
F 0 "C11" H 5565 6471 50  0000 L CNN
F 1 "C_1uF" H 5565 6380 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 5488 6275 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2018/06/20/CL21A105KAFNNNE.pdf" H 5475 6525 50  0001 C CNN
F 4 "DK" H 5450 6425 60  0001 C CNN "MFN"
F 5 "1276-2887-1-ND" H 5450 6425 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/product-detail/en/samsung-electro-mechanics/CL21A105KAFNNNE/1276-2887-1-ND/3890973" H 5875 6925 60  0001 C CNN "PurchasingLink"
	1    5450 6425
	1    0    0    -1  
$EndComp
$Comp
L footprints:C_0.1uF C7
U 1 1 5DFB13CD
P 5750 6425
F 0 "C7" H 5865 6471 50  0000 L CNN
F 1 "C_0.1uF" H 5865 6380 50  0000 L CNN
F 2 "footprints:C_0805_OEM" H 5788 6275 50  0001 C CNN
F 3 "http://datasheets.avx.com/X7RDielectric.pdf" H 5775 6525 50  0001 C CNN
F 4 "DK" H 5750 6425 60  0001 C CNN "MFN"
F 5 "478-3352-1-ND" H 5750 6425 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=478-3352-1-ND" H 6175 6925 60  0001 C CNN "PurchasingLink"
	1    5750 6425
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 6275 5450 6275
Connection ~ 5300 6275
Wire Wire Line
	5450 6275 5750 6275
Connection ~ 5450 6275
Wire Wire Line
	5175 6575 5450 6575
Connection ~ 5450 6575
Wire Wire Line
	5450 6575 5750 6575
Wire Wire Line
	5750 6575 5750 6650
Connection ~ 5750 6575
$Comp
L power:GND #PWR045
U 1 1 5DFC6599
P 5750 6650
F 0 "#PWR045" H 5750 6400 50  0001 C CNN
F 1 "GND" H 5755 6477 50  0000 C CNN
F 2 "" H 5750 6650 50  0001 C CNN
F 3 "" H 5750 6650 50  0001 C CNN
	1    5750 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 6075 5675 6075
Connection ~ 5300 6075
$Comp
L footprints:R_0_2512 R9
U 1 1 5DFCD18F
P 5825 6075
F 0 "R9" V 5618 6075 50  0000 C CNN
F 1 "R_0_2512" V 5709 6075 50  0000 C CNN
F 2 "footprints:R_2512_OEM" H 5755 6075 50  0001 C CNN
F 3 "http://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=9-1773463-7&DocType=DS&DocLang=English" H 5905 6075 50  0001 C CNN
F 4 "DK" H 5825 6075 60  0001 C CNN "MFN"
F 5 "A121322CT-ND" H 5825 6075 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=A121322CT-ND" H 6305 6475 60  0001 C CNN "PurchasingLink"
	1    5825 6075
	0    1    1    0   
$EndComp
Wire Wire Line
	5975 6075 6150 6075
Wire Wire Line
	6150 6075 6150 5875
$Comp
L power:VCC #PWR046
U 1 1 5DFD3788
P 6150 5875
F 0 "#PWR046" H 6150 5725 50  0001 C CNN
F 1 "VCC" H 6167 6048 50  0000 C CNN
F 2 "" H 6150 5875 50  0001 C CNN
F 3 "" H 6150 5875 50  0001 C CNN
	1    6150 5875
	1    0    0    -1  
$EndComp
$Comp
L footprints:R_200 R12
U 1 1 5DFD428E
P 6150 6225
F 0 "R12" H 6220 6271 50  0000 L CNN
F 1 "R_200" H 6220 6180 50  0000 L CNN
F 2 "footprints:R_0805_OEM" H 6080 6225 50  0001 C CNN
F 3 "https://www.seielect.com/Catalog/SEI-RMCF_RMCP.pdf" H 6230 6225 50  0001 C CNN
F 4 "DK" H 6150 6225 60  0001 C CNN "MFN"
F 5 "RMCF0805JT200RCT-ND" H 6150 6225 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=RMCF0805JT200RCT-ND" H 6630 6625 60  0001 C CNN "PurchasingLink"
	1    6150 6225
	1    0    0    -1  
$EndComp
Connection ~ 6150 6075
$Comp
L footprints:LED_0805 D2
U 1 1 5DFD54C1
P 6150 6525
F 0 "D2" V 6189 6408 50  0000 R CNN
F 1 "LED_0805" V 6098 6408 50  0000 R CNN
F 2 "footprints:LED_0805_OEM" H 6050 6525 50  0001 C CNN
F 3 "http://www.osram-os.com/Graphics/XPic9/00078860_0.pdf" H 6150 6625 50  0001 C CNN
F 4 "DK" H 6150 6525 60  0001 C CNN "MFN"
F 5 "475-1410-1-ND" H 6150 6525 60  0001 C CNN "MPN"
F 6 "https://www.digikey.com/products/en?keywords=475-1410-1-ND" H 6550 7025 60  0001 C CNN "PurchasingLink"
	1    6150 6525
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR047
U 1 1 5DFD6E78
P 6150 6675
F 0 "#PWR047" H 6150 6425 50  0001 C CNN
F 1 "GND" H 6155 6502 50  0000 C CNN
F 2 "" H 6150 6675 50  0001 C CNN
F 3 "" H 6150 6675 50  0001 C CNN
	1    6150 6675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR041
U 1 1 5DFD7C11
P 2950 7200
F 0 "#PWR041" H 2950 6950 50  0001 C CNN
F 1 "GND" H 2955 7027 50  0000 C CNN
F 2 "" H 2950 7200 50  0001 C CNN
F 3 "" H 2950 7200 50  0001 C CNN
	1    2950 7200
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5DFDD84E
P 1500 7100
F 0 "#FLG01" H 1500 7175 50  0001 C CNN
F 1 "PWR_FLAG" H 1500 7273 50  0000 C CNN
F 2 "" H 1500 7100 50  0001 C CNN
F 3 "~" H 1500 7100 50  0001 C CNN
	1    1500 7100
	-1   0    0    1   
$EndComp
$Comp
L power:+9V #PWR024
U 1 1 5DFDFCBE
P 1500 7100
F 0 "#PWR024" H 1500 6950 50  0001 C CNN
F 1 "+9V" H 1515 7273 50  0000 C CNN
F 2 "" H 1500 7100 50  0001 C CNN
F 3 "" H 1500 7100 50  0001 C CNN
	1    1500 7100
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR040
U 1 1 5DFE0DA5
P 2950 5775
F 0 "#PWR040" H 2950 5625 50  0001 C CNN
F 1 "+9V" H 2965 5948 50  0000 C CNN
F 2 "" H 2950 5775 50  0001 C CNN
F 3 "" H 2950 5775 50  0001 C CNN
	1    2950 5775
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR08
U 1 1 5DFE1EDE
P 1425 5950
F 0 "#PWR08" H 1425 5800 50  0001 C CNN
F 1 "VCC" H 1442 6123 50  0000 C CNN
F 2 "" H 1425 5950 50  0001 C CNN
F 3 "" H 1425 5950 50  0001 C CNN
	1    1425 5950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR050
U 1 1 5DFE2ADC
P 7125 4650
F 0 "#PWR050" H 7125 4500 50  0001 C CNN
F 1 "VCC" H 7142 4823 50  0000 C CNN
F 2 "" H 7125 4650 50  0001 C CNN
F 3 "" H 7125 4650 50  0001 C CNN
	1    7125 4650
	1    0    0    -1  
$EndComp
$Comp
L footprints:CONN_01x02 J11
U 1 1 5DFE4184
P 5975 7400
F 0 "J11" H 6055 7392 50  0000 L CNN
F 1 "CONN_01x02" H 6055 7301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5875 7400 50  0001 C CNN
F 3 "https://katalog.we-online.de/em/datasheet/6130xx11121.pdf" H 5975 7500 50  0001 C CNN
F 4 "DK" H 6075 7600 50  0001 C CNN "MFN"
F 5 "732-5315-ND" H 6175 7700 50  0001 C CNN "MPN"
	1    5975 7400
	1    0    0    -1  
$EndComp
$Comp
L power:+9V #PWR048
U 1 1 5DFE51FE
P 5775 7400
F 0 "#PWR048" H 5775 7250 50  0001 C CNN
F 1 "+9V" H 5790 7573 50  0000 C CNN
F 2 "" H 5775 7400 50  0001 C CNN
F 3 "" H 5775 7400 50  0001 C CNN
	1    5775 7400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR049
U 1 1 5DFE5BF9
P 5775 7500
F 0 "#PWR049" H 5775 7250 50  0001 C CNN
F 1 "GND" H 5780 7327 50  0000 C CNN
F 2 "" H 5775 7500 50  0001 C CNN
F 3 "" H 5775 7500 50  0001 C CNN
	1    5775 7500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 5DFE803B
P 3175 1775
F 0 "#PWR0101" H 3175 1625 50  0001 C CNN
F 1 "VCC" H 3192 1948 50  0000 C CNN
F 2 "" H 3175 1775 50  0001 C CNN
F 3 "" H 3175 1775 50  0001 C CNN
	1    3175 1775
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0102
U 1 1 5DFE8ECC
P 3250 1775
F 0 "#PWR0102" H 3250 1625 50  0001 C CNN
F 1 "VCC" H 3267 1948 50  0000 C CNN
F 2 "" H 3250 1775 50  0001 C CNN
F 3 "" H 3250 1775 50  0001 C CNN
	1    3250 1775
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0103
U 1 1 5DFE9C0B
P 7225 1975
F 0 "#PWR0103" H 7225 1825 50  0001 C CNN
F 1 "VCC" H 7242 2148 50  0000 C CNN
F 2 "" H 7225 1975 50  0001 C CNN
F 3 "" H 7225 1975 50  0001 C CNN
	1    7225 1975
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0104
U 1 1 5DFEAAFF
P 7300 1975
F 0 "#PWR0104" H 7300 1825 50  0001 C CNN
F 1 "VCC" H 7317 2148 50  0000 C CNN
F 2 "" H 7300 1975 50  0001 C CNN
F 3 "" H 7300 1975 50  0001 C CNN
	1    7300 1975
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0105
U 1 1 5DFEBCE4
P 10275 5100
F 0 "#PWR0105" H 10275 4950 50  0001 C CNN
F 1 "VCC" V 10293 5227 50  0000 L CNN
F 2 "" H 10275 5100 50  0001 C CNN
F 3 "" H 10275 5100 50  0001 C CNN
	1    10275 5100
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR0106
U 1 1 5DFECD6F
P 10300 5525
F 0 "#PWR0106" H 10300 5375 50  0001 C CNN
F 1 "VCC" V 10318 5652 50  0000 L CNN
F 2 "" H 10300 5525 50  0001 C CNN
F 3 "" H 10300 5525 50  0001 C CNN
	1    10300 5525
	0    -1   -1   0   
$EndComp
NoConn ~ 10300 5625
$Comp
L power:VCC #PWR0107
U 1 1 5DFF9F05
P 3250 2500
F 0 "#PWR0107" H 3250 2350 50  0001 C CNN
F 1 "VCC" H 3267 2673 50  0000 C CNN
F 2 "" H 3250 2500 50  0001 C CNN
F 3 "" H 3250 2500 50  0001 C CNN
	1    3250 2500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0108
U 1 1 5DFFAB0C
P 7300 2700
F 0 "#PWR0108" H 7300 2550 50  0001 C CNN
F 1 "VCC" H 7317 2873 50  0000 C CNN
F 2 "" H 7300 2700 50  0001 C CNN
F 3 "" H 7300 2700 50  0001 C CNN
	1    7300 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2500 6950 2500
Wire Wire Line
	6500 2600 6950 2600
Wire Wire Line
	2450 2300 2900 2300
Wire Wire Line
	2450 2400 2900 2400
$Comp
L formula:CONN_01X03 J12
U 1 1 5E16DFDE
P 10500 3900
F 0 "J12" H 10528 3941 50  0000 L CNN
F 1 "CONN_01X03" H 10528 3850 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10500 2700 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10500 2700 50  0001 C CNN
F 4 "Mouser" H 10500 3900 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10500 3900 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10900 4500 60  0001 C CNN "PurchasingLink"
	1    10500 3900
	1    0    0    -1  
$EndComp
$Comp
L formula:CONN_01X03 J13
U 1 1 5E16DFE7
P 10500 4225
F 0 "J13" H 10528 4266 50  0000 L CNN
F 1 "CONN_01X03" H 10528 4175 50  0000 L CNN
F 2 "footprints:MHSS1105" H 10500 3025 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/181/M20-999-1218971.pdf" H 10500 3025 50  0001 C CNN
F 4 "Mouser" H 10500 4225 60  0001 C CNN "MFN"
F 5 "855-M20-9990346" H 10500 4225 60  0001 C CNN "MPN"
F 6 "https://www.mouser.com/ProductDetail/Harwin/M20-9990346?qs=sGAEpiMZZMs%252bGHln7q6pmzlZUuX%2f53qj1ROyRKct5o4%3d" H 10900 4825 60  0001 C CNN "PurchasingLink"
	1    10500 4225
	1    0    0    -1  
$EndComp
Text Label 10250 3900 2    50   ~ 0
TX
Text Label 10250 4000 2    50   ~ 0
RX
$Comp
L power:GND #PWR013
U 1 1 5E16DFEF
P 10250 4125
F 0 "#PWR013" H 10250 3875 50  0001 C CNN
F 1 "GND" V 10255 3997 50  0000 R CNN
F 2 "" H 10250 4125 50  0001 C CNN
F 3 "" H 10250 4125 50  0001 C CNN
	1    10250 4125
	0    1    1    0   
$EndComp
NoConn ~ 10250 3800
NoConn ~ 10250 4325
$Comp
L power:VCC #PWR014
U 1 1 5E16DFF7
P 10250 4225
F 0 "#PWR014" H 10250 4075 50  0001 C CNN
F 1 "VCC" V 10268 4352 50  0000 L CNN
F 2 "" H 10250 4225 50  0001 C CNN
F 3 "" H 10250 4225 50  0001 C CNN
	1    10250 4225
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
