'''
Class for controlling DC motor, Servo, and HC-SR04 Ultrasonic Sensor
Please calibrate your servo before using the class.
'''

import sys
import time
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from gpiozero import AngularServo
from gpiozero import InputDevice
import math
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class DCmotor:
	def __init__(self,INA1=None,INA2=None,EN=None,ENCODA=None,ENCODB=None,use_encoder=False,use_motor=False):
		self._INA1 = INA1
		self._INA2 = INA2
		self._EN = EN
		self._ENCODA = ENCODA
		self._ENCODB = ENCODB
		if use_motor:
			self._INA1 = INA1
			self._INA2 = INA2
			self._EN = EN
			GPIO.setup(self._EN, GPIO.OUT)
			GPIO.setup(self._INA1, GPIO.OUT)
			GPIO.setup(self._INA2, GPIO.OUT)
			self.pwm = GPIO.PWM(self._EN, 10) #50Hz
			self.pwm.ChangeFrequency(4096)
			self.pwm.start(0)
			self.direction = True
			GPIO.output(self._INA1, True)
			GPIO.output(self._INA2, False)
			GPIO.output(self._EN, True)
		if use_encoder:
			self._ENCODA = ENCODA
			self._ENCODB = ENCODB
			GPIO.setup(self._ENCODA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			GPIO.setup(self._ENCODB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
			GPIO.add_event_detect(self._ENCODA, GPIO.RISING, callback=self.update_encoder_ticks)
			self._ticks = 0
			self.seq = self.rotation_seq()
			self.last_delta = 0

	def rotation_seq(self,):
		a,b = GPIO.input(self._ENCODA), GPIO.input(self._ENCODB)
		return (a^b) | b << 1
		
		

	def change_duty_cycle(self,duty=0):
		# self.pwm.ChangeDutyCycle(abs(duty))
		if (duty>=0) == self.direction: 
			self.pwm.ChangeDutyCycle(abs(duty))
		else: 
			self.direction = not self.direction
			GPIO.output(self._INA1, self.direction)
			GPIO.output(self._INA2, not self.direction)
			self.pwm.ChangeDutyCycle(abs(duty))
		
	def close_channel(self):
		GPIO.output(self._INA1, False)
		GPIO.output(self._INA2, False)
		GPIO.output(self._EN, False)
		self.pwm.ChangeDutyCycle(0)
		
	def forward(self,dutycycle,):
		
		GPIO.output(self._INA1, True)
		GPIO.output(self._INA2, False)
		GPIO.output(self._EN, True)
		self.change_duty_cycle(dutycycle)
		
	def reverse(self,dutycycle,):
		
		GPIO.output(self._EN, True)
		GPIO.output(self._INA1, False)
		GPIO.output(self._INA2, True)
		self.change_duty_cycle(dutycycle)
	
	def request_encoder_readings(self):
		return GPIO.input(self._ENCODA), GPIO.input(self._ENCODB)
	
	def update_encoder_ticks(self, channel):
		# delta = 0
		# seq = self.rotation_seq()
		# if seq != self.seq:
		# 	delta = (seq - self.seq)%4
		# 	if delta == 3:
		# 		delta = -1
		# 	elif delta == 2:
		# 		delta = int(math.copysign(delta,self.last_delta))
		# 	self.last_delta = delta
		# 	self.seq = seq
		# self._ticks += delta
		if (GPIO.input(self._ENCODB) != GPIO.input(self._ENCODA)):
			self._ticks += 1
		else:
			self._ticks -= 1
				
	def request_encoder_ticks(self):
		return self._ticks

	# Call this function to release the pins
	def cleanup(self,):
		if use_motor:
			self.pwm.stop()
			GPIO.cleanup([self._INA2,self._INA1,self._EN])
		if use_encoder:
			GPIO.cleanup([self._ENCODA,self._ENCODB])
		
class Ultrasonic():
	def __init__(self,max_distance=3, threshold_distance =0.01, echo=17, trigger=18):
		self._echo = echo
		self._trigger = trigger
		self.ultrasonic = DistanceSensor(
							echo=self._echo, 
							trigger=self._trigger,
							max_distance = max_distance,
							threshold_distance = threshold_distance
							)
	
	def request_distance(self):
		return self.ultrasonic.distance
		
class Servo():
	def __init__(self,pin = 17, min_angle=-42, max_angle=44):
		self.servo = AngularServo(pin, min_angle, max_angle)
	
	def right(self):
		self.servo = 90
		
	def forward():
		self.servo = 0
		
	def left():
		self.servo = -90
	
def main():
	# sensor = Ultrasonic()
	# i = 0
	# while i < 100:
	# 	time.sleep(1)
	# 	print sensor.request_distance()*100
	# 	i+=1
	# GPIO.cleanup()
	pass


if __name__ == '__main__':
	main()
