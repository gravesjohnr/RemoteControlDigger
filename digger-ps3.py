import time
import signal
from pathlib import Path

from gpiozero import LED
def handler(signum, frame):
    pass

try:
    signal.signal(signal.SIGHUP, handler)
except AttributeError:
    # Windows compatibility
    pass

print('starting')
tailLightLED = LED(17) # Choose the correct pin number
tailLightLEDOn = False
headLightLED = LED(22) # Choose the correct pin number
headLightLEDOn = False
topLightLED = LED(16) # Choose the correct pin number
topLightLEDOn = False

# Sound Toggles
engineStarted = False
engineVolume = 5
musicPlaying = -1
musicVolume = 5
beepVolume = 5
beepPlaying = False

from darkwater_640 import dw_Controller, dw_Motor

import pygame

import os
os.environ["SDL_VIDEODRIVER"] = "dummy"


# Initialise the pygame library
pygame.init()
try:
  pygame.mixer.init()
except pygame.error:
  print "Not able to deal with audio.  skipping"
  pass

pygame.display.set_mode((600, 400)) # Set dummy display

#pygame.mixer.init()
print("Pygame init done.")

joystickAvailable=False
print "Number of controllers found: %d" % pygame.joystick.get_count()
while pygame.joystick.get_count() == 0:
  print("ps3 controller not connected.")
  time.sleep(5)
  exit()


# Connect to the first JoyStick
j = pygame.joystick.Joystick(0)
j.init()

print 'Initialized Joystick : %s' % j.get_name()

dw = dw_Controller( addr=0x60 )
m1 = dw.getMotor(1)
m2 = dw.getMotor(2)
m3 = dw.getMotor(3)
m4 = dw.getMotor(4)
m5 = dw.getMotor(5)
m6 = dw.getMotor(6)

m1.off()
m2.off()
m3.off()
m4.off()
m5.off()
m6.off()

# Servos
s1 = dw.getServo(1)
s1.off()
s1.setPWMuS(1000) # set fully counter clockwise

s2 = dw.getServo(2)
s2.off()
s2.setPWMuS(1000) # set fully counter clockwise

time.sleep(1)

print 'Motors initilized'
print 'pygame.JOYAXISMOTION: %d' % pygame.JOYAXISMOTION
print 'pygame.JOYBUTTONDOWN: %d' % pygame.JOYBUTTONDOWN

while True:
  events = pygame.event.get()
  for event in events:
    #print 'type: %d' % event.type

    if event.type == pygame.JOYAXISMOTION:
      if event.axis == 0: # Left/right on left stick (Rotate)
        value=-(event.value*255)
        if value < 5 and value > -5:
          value = 0
        #print 'axis 0 left/right %d' % (value)
        m2.setMotorSpeed(int(value))
      if event.axis == 1: # Up/Down on left stick (Large arm)
        value=(event.value*255)
        if value < 5 and value > -5:
          value = 0
        #print 'axis 1 up/down %d' % (value)
        m1.setMotorSpeed(int(value))
      if event.axis == 2: # Left/Right on right stick (Small arm)
        value=-(event.value*255)
        if value < 5 and value > -5:
          value = 0
          #print 'axis 2 %d' % (value)
        m6.setMotorSpeed(int(value))
      if event.axis == 3: #  Up/Down on right stick (Bucket)
        value=-(event.value*255)
        if value < 5 and value > -5:
          value = 0
          #print 'axis 3 %d' % (value)
        m5.setMotorSpeed(int(value))
      if event.axis == 12: # Left trigger bottom button
        value=-(event.value*127)
        value = value + 127
        value = 255 - value
        value = -(value)
        if value < 5 and value > -5:
          value = 0
        print 'axis 12 trigger %d' % (value)
        m3.setMotorSpeed(int(value))
      if event.axis == 14: # Left trigger top button Reverse
        value=-(event.value*127)
        value = value + 127
        value = 255 - value
        if value < 5 and value > -5:
          value = 0
        print 'axis 14 trigger %d' % (value)
        m3.setMotorSpeed(int(value))
        if value == 0:
          pygame.mixer.Channel(2).stop()
          beepPlaying = False
        else:
          if beepPlaying == False:
            beepPlaying = True
            pygame.mixer.Channel(2).play(pygame.mixer.Sound('/home/pi/RemoteControlDigger/beep.wav'))
            pygame.mixer.Channel(2).set_volume(engineVolume*.1)
      if event.axis == 15: # Right trigger top button Reverse
        value=-(event.value*127)
        value = value + 127
        value = 255 - value
        value = -(value)
        if value < 5 and value > -5:
          value = 0
        print 'axis 15 trigger %d' % (value)
        m4.setMotorSpeed(int(value))
        if value == 0:
          pygame.mixer.Channel(2).stop()
          beepPlaying = False
        else:
          if beepPlaying == False:
            beepPlaying = True
            pygame.mixer.Channel(2).play(pygame.mixer.Sound('/home/pi/RemoteControlDigger/beep.wav'))
            pygame.mixer.Channel(2).set_volume(engineVolume*.1)
      if event.axis == 13: # Right trigger bottom button
        value=-(event.value*127)
        value = value + 127
        value = 255 - value
        if value < 5 and value > -5:
          value = 0
        print 'axis 13 trigger %d' % (value)
        m4.setMotorSpeed(int(value))

    if event.type == pygame.JOYBUTTONDOWN:
      print 'Button press %d' % (event.button)
      if event.button == 8: # Left button
        print 'button 8'
      if event.button == 3: # Start button
        print 'button 3'
        if engineStarted == True:
          pygame.mixer.Channel(0).stop()
          engineStarted = False
        else:
          pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/pi/RemoteControlDigger/EngineStart.wav'))
          pygame.mixer.Channel(0).set_volume(engineVolume*.1)
          time.sleep(1)     # wait a second
          pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/pi/RemoteControlDigger/EngineRun.wav'),-1)
          engineStarted = True
      if event.button == 0: # Select Button Play music from the songs directory
        musicPlaying = musicPlaying+1
        songFile = "/home/pi/RemoteControlDigger/songs/"+str(musicPlaying)+".wav"
        if Path(songFile).exists() == True:
          pygame.mixer.Channel(1).play(pygame.mixer.Sound(songFile))
          pygame.mixer.Channel(1).set_volume(musicVolume*.1)
        else:
          pygame.mixer.Channel(1).stop()
          musicPlaying = -1
      if event.button == 12:
        print 'button 12'
        if headLightLEDOn:
          headLightLED.off()
          headLightLEDOn=False
        else:
          headLightLED.on()
          headLightLEDOn=True
      if event.button == 13:
        print 'button 13'
        if tailLightLEDOn:
          tailLightLED.off()
          tailLightLEDOn=False
        else:
          tailLightLED.on()
          tailLightLEDOn=True
      if event.button == 15:
        if topLightLEDOn:
          print 'top light off'
          topLightLED.off()
          topLightLEDOn=False
        else:
          print 'top light on'
          topLightLED.on()
          topLightLEDOn=True
      if event.button == 14:
        print 'Setting flashing yellow light'
        s1.setPWMuS(2000) # Set fully clockwise
        time.sleep(1)     # wait a second
        s1.setPWMuS(1000) # Set fully counter clockwise
      if event.button == 4: # Music Volume Up
        musicVolume=musicVolume+1
        if musicVolume > 10:
          musicVolume=10
        print("Setting volume to "+str(musicVolume))
        pygame.mixer.Channel(1).set_volume(musicVolume*.1)
      if event.button == 6: # Music Volume down
        musicVolume=musicVolume-1
        if musicVolume < 0:
          musicVolume=0
        print("Setting volume to "+str(musicVolume))
        pygame.mixer.Channel(1).set_volume(musicVolume*.1)
      if event.button == 5: # Engine Volume Up
        engineVolume=engineVolume+1
        if engineVolume > 10:
          engineVolume=10
        print("Setting volume to "+str(engineVolume))
        pygame.mixer.Channel(0).set_volume(engineVolume*.1)
      if event.button == 7: # Engine Volume down
        engineVolume=engineVolume-1
        if engineVolume < 0:
          engineVolume=0
        print("Setting volume to "+str(engineVolume))
        pygame.mixer.Channel(0).set_volume(engineVolume*.1)
        

m1.off()
m2.off()
m3.off()
m4.off()
m5.off()
m6.off()
