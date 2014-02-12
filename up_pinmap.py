HIGH = 1
LOW = 0

INPUT_PULLUP = 4
INPUT_PULLDOWN = 2
INPUT_HIGHZ = 0
ADC_INPUT = 5
OUTPUT = 1
PWM0_OUTPUT = 6
PWM1_OUTPUT = 7
PWM_OUTPUT = 8

DEFAULT_INPUT_MODE = INPUT_PULLUP
DEFAULT_ADC_PINMODE = INPUT_HIGHZ
PWM_DEFAULT_POLARITY = LOW

PRIMARY_PIN_OUTPUT = 2
PRIMARY_PIN_INPUT = 1
SECONDARY_PIN = 0

UPER_HARD_INTERRUPTS = 8

# pinmap for UPER1 board
pins = []
pins.append("none")	#0 - no zero pin on UPER1
pins.append(0) 		#1 	PIO0_20
pins.append(1) 		#2 	PIO0_2
pins.append(2) 		#3 	PIO1_26 PWM1_2
pins.append(3) 		#4 	PIO1_27
pins.append(4) 		#5 	PIO1_20 SPI1_SCK
pins.append("SCL")	#6 	PIO0_4 - pin is dedicated to I2C function exclusivelly
pins.append("SDA")	#7 	PIO0_5 - pin is dedicated to I2C function exclusivelly
pins.append(5) 		#8 	PIO0_21 SPI1_MOSI
pins.append(6)		#9 	PIO1_23
pins.append(7)		#10	PIO1_24	PWM1_0
pins.append(8)		#11	PIO0_7
pins.append(9)		#12	PIO1_28
pins.append(10)		#13	PIO1_31
pins.append(11)		#14	PIO1_21 SPI1_MISO
pins.append(12)		#15	PIO0_8	SPI0_MISO
pins.append(13)		#16	PIO0_9	SPI0_MOSI
pins.append(14)		#17	PIO0_10	SPI0_SCK
pins.append(15)		#18	PIO1_29
pins.append("+5V")	#19 +5V DC out
pins.append("GND")	#20 ground

pins.append("GND")	#21 ground
pins.append("+3.3V")#22 +3.3V DC out
pins.append(33)		#23	PIO0_11	ADC0
pins.append(32)		#24	PIO0_12	ADC1
pins.append(31)		#25	PIO0_13	ADC2
pins.append(30)		#26	PIO0_14	ADC3
pins.append(29)		#27	PIO1_13	PWM0_0
pins.append(28)		#28	PIO1_14	PWM0_1
pins.append(27)		#29	PIO1_22
pins.append(26)		#30	PIO0_15	ADC4
pins.append(25)		#31	PIO0_16	ADC5
pins.append(24)		#32	PIO0_22	ADC6
pins.append(23)		#33	PIO0_23	ADC7
pins.append(22)		#34	PIO1_15	PWM0_2
pins.append(21)		#35	PIO0_17
pins.append(20)		#36	PIO0_18	UART_RX
pins.append(19)		#37	PIO0_19	UART_TX
pins.append(18)		#38	PIO0_16
pins.append(17)		#39	PIO1_25 PWM1_1
pins.append(16)		#50	PIO1_19

# UPER adc pins
adcs = []
adcs.append(33)		#23 ADC0
adcs.append(32)		#24 ADC1
adcs.append(31)		#25 ADC2
adcs.append(30)		#26 ADC3
adcs.append(26)		#30	ADC4
adcs.append(25)		#31 ADC5
adcs.append(24)		#32	ADC6
adcs.append(23)		#33 ADC7

# UPER pwm pins
pwms = []
pwms.append(29)		#27	PWM0_0
pwms.append(28)		#28 PWM0_1
pwms.append(22)		#34 PWM0_2
pwms.append(7)		#10 PWM1_0
pwms.append(17)		#39 PWM1_1
pwms.append(2)		#3  PWM1_2

spi1 = []
spi1.append(14)		#SPI1_MISO
spi1.append(8)		#SPI1_MOSI
spi1.append(5)		#SPI1_SCK

spi0 = []
spi0.append(15)		#SPI1_MISO
spi0.append(16)		#SPI1_MISO
spi0.append(17)		#SPI1_MISO

LEDPIN_R = 27		#PWMPIN0_0
LEDPIN_G = 28		#PWMPIN0_1
LEDPIN_B = 34		#PWMPIN0_2

# Interrupt modes
# HIGH and LOW were already declared
# LOW 0 
# HIGH 1
CHANGE = 2
RISING = 3
FALLING = 4

# String as response from LPC, this replaces numbers 0-4
interruptType = []
interruptType.append("LOW")
interruptType.append("HIGH")
interruptType.append("CHANGE")
interruptType.append("RISING")
interruptType.append("FALLING")

# number of hard interrupts
HARD_INTERRUPTS = 8