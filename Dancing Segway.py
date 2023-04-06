import pyb
import time
from pyb import Pin, Timer, ADC, DAC, LED
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE
from neopixel import NeoPixel
# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

f = open('dance.txt', 'r') #opening file dance.txt as read
line = f.read().split(",") # accessing text file for dance moves and splitting to access each step
f = open('dance2.txt', 'r') #opening file dance.txt as read
line2 = f.read().split(",") # accessing text file for dance moves and splitting to access each step
# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Lab 5 - Task 3b')
oled.display()

def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)

def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
	
def A_stop():
	A1.high()
	A2.high()
	
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def B_stop():
	B1.high()
	B2.high()
def forward(value):
	A_forward(value)
	B_forward(value)
def backward(value):
	A_back(value)
	B_back(value)

def left(value):
	A_forward(value)
	B_back(value)
	

def right(value):
	A_back(value)
	B_forward(value)

def forward(value):
	A_forward(value)
	B_forward(value)


# Initialise variables
speed = 0
A_speed = 0
A_count = 0
B_speed = 0
B_count = 0

#-------  Section to set up Interrupts ----------
def isr_motorA(dummy):	# motor A sensor ISR - just count transitions
	global A_count
	A_count += 1

def isr_motorB(dummy):	# motor B sensor ISR - just count transitions
	global B_count
	B_count += 1
		
def isr_speed_timer(dummy): 	# timer interrupt at 100msec intervals
	global A_count
	global A_speed
	global B_count
	global B_speed
	A_speed = A_count			# remember count value
	B_speed = B_count
	A_count = 0					# reset the count
	B_count = 0


#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Beat Detection')
oled.display()

# define ports for microphone, LEDs and trigger out (X5)

b_LED = LED(4)		# flash for beats on blue LED
start=0
end=0
time_elapsed =0
def flash():		# routine to flash blue LED when beat detected
	b_LED.on()
	pyb.delay(10)
	b_LED.off()
	

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 30	# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
tic = pyb.millis()			# mark time now in msec
step = True
# create neopixel object
np = NeoPixel(Pin("Y12", Pin.OUT), 8)
while True:				# Main program loop
	if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full
		
		# Fetch instantaneous energy
		E = audio.inst_energy()			# fetch instantenous energy
		audio.reset_buffer()			# get ready for next epoch

		# compute moving sum of last 50 energy epochs with circular buffer
		sum_energy = sum_energy - e_buf[e_ptr] + E
		e_buf[e_ptr] = E			# over-write earliest energy with most recent
		e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
		average_energy = sum_energy/M

		# Compute ratio of instantaneous energy/average energy
		c = (E/average_energy)*10
		counter = 0
		oled.draw_text(0,30,str(c))
		oled.display()
		if (pyb.millis()-tic > MIN_BEAT_PERIOD):# if longer than minimum period
			for i in range (0, len(line)):
				if (c>10 and c<60) and step == True:
					
					if line[i] == "L" and counter % 2 == 0:
						left(30)
						for i in range(8):  # turn LEDs purple on
							np[i] = (219,62,177)
							np.write()
							
						flash()
						oled.draw_text(0,40, 'left')	# Useful to show what's happening?
						oled.display()
						counter += 1
						#pyb.delay(100)
					elif line[i] == "R" and counter%2 == 1:
						right(30)
						for i in range(8):  # turn LEDs yellow one at a time
							np[i] = (255,173,0)
							np.write()
						
						oled.draw_text(0,50, 'right')	# Useful to show what's happening?
						oled.display()
						counter += 1
						#pyb.delay(100)
					
					elif line[i] == "S" :
						forward(0)
						oled.draw_text(0,60, 'stop')	# Useful to show what's happening?
						oled.display()
						#step = False
						
				else:
					backward(40)

			for j in range (0, len(line2)):
				if (c>10)and step == True:
					
					if line2[j] == "F" and counter % 2 == 0:
						forward(30)
						for i in range(8):  # turn LEDs red one at a time
							np[i] = (210,39,48)
							np.write()
						flash()
						oled.draw_text(0,40, 'forward')	# Useful to show what's happening?
						oled.display()
						counter += 1
						#pyb.delay(100)
					elif line2[j] == "B" and counter%2 == 1:
						backward(30)
						for i in range(8):  # turn LEDs blue one at a time
							np[i] = (77, 77, 255)
							np.write()
						oled.draw_text(0,50, 'backward')	# Useful to show what's happening?
						oled.display()
						counter += 1
						#pyb.delay(100)
					
					elif line2[j] == "S" :
						forward(0)
						oled.draw_text(0,60, 'stop')	# Useful to show what's happening?
						oled.display()
						#step = False
				else:
					forward(40)
			
				tic = pyb.millis()		# reset tic	'''
		buffer_full = False				# reset status flag