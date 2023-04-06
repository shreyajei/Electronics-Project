
import pyb
from pyb import LED, Timer
from oled_938 import OLED_938
from mpu6050 import MPU6050

# Define LEDs
b_LED = LED(4)
g_pitch = 0 
g_roll =0   
# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()

# IMU connected to X9 and X10
imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard

def read_imu(dt):
	
	global g_pitch
	global g_roll
	alpha = 0.7    # larger = longer time constant
	pitch = int(imu.pitch())
	roll = int(imu.roll())
	g_pitch = alpha*(g_pitch + imu.get_gy()*dt*0.001) + (1-alpha)*pitch
	g_roll = alpha*(g_roll + imu.get_gx()*dt*0.001) + (1-alpha)*roll
	# show graphics
	oled.clear()
	#oled.line(96, 26, pitch, 24, 1)
	#oled.line(32, 26, g_pitch, 24, 1)
	
	#oled.line(96, 26, roll, 34, 1)
	#oled.line(12, 26, g_roll, 24, 1)
	#oled.display()
tic = pyb.millis()		


# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

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
	
# Create external interrupts for motorA Hall Effect Senor
import micropython
micropython.alloc_emergency_exception_buf(100)
from pyb import ExtInt

motorA_int = ExtInt ('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorA)
motorB_int = ExtInt ('Y6', ExtInt.IRQ_RISING, Pin.PULL_NONE,isr_motorB)

# Create timer interrupts at 100 msec intervals
speed_timer = pyb.Timer(4, freq=10)
speed_timer.callback(isr_speed_timer)

#-------  END of Interrupt Section  ----------

while True:				# loop forever until CTRL-C
	
	# drive motor - controlled by potentiometer
	speed = int((pot.read()-2048)*200/4096)

	if (g_pitch >= 0):		# forward
		A_forward(g_pitch)
		B_forward(g_roll)

	else:
		A_back(abs(g_pitch))
		B_back(abs(g_roll))
	b_LED.toggle()
	toc = pyb.millis()
	read_imu(toc-tic)
	tic = pyb.millis()
	# Display new speed
	#oled.draw_text(0,20,'Motor Drive:{:5d}%'.format(speed))
	oled.draw_text(0,15,'Motor A:{:5.2f} rps'.format(A_speed/39))	
	oled.draw_text(0,25,'Motor B:{:5.2f} rps'.format(B_speed/39))
	oled.draw_text(0,35,'Pitch:{:5.2f} '.format(g_pitch))
	oled.draw_text(0,45,'Roll:{:5.2f} '.format(g_roll))
	oled.display()
	
	pyb.delay(100)
