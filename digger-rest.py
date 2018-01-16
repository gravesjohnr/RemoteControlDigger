import time
from darkwater_640 import dw_Controller, dw_Motor
import jsonify

dw = dw_Controller( addr=0x60 )
m1 = dw.getMotor(1)
m2 = dw.getMotor(2)
m3 = dw.getMotor(3)
m4 = dw.getMotor(4)
m5 = dw.getMotor(5)
m6 = dw.getMotor(6)
time.sleep(1.0)
print 'Motors initilized'


from flask import Flask
app = Flask(__name__)

@app.route("/mainarm/up/<int:speed>/<int:howlong>", methods=['POST'])
def mainarmUp(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   speed = -(speed)
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m1.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m1.setMotorSpeed(0)
   return "ok"

@app.route("/mainarm/down/<int:speed>/<int:howlong>", methods=['POST'])
def mainarmDown(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m1.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m1.setMotorSpeed(0)
   return "ok"

@app.route("/rotate/left/<int:speed>/<int:howlong>", methods=['POST'])
def rotateLeft(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m2.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m2.setMotorSpeed(0)
   return "ok"

@app.route("/rotate/right/<int:speed>/<int:howlong>", methods=['POST'])
def rotateRight(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   speed = -(speed)
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m2.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m2.setMotorSpeed(0)
   return "ok"

@app.route("/smallarm/up/<int:speed>/<int:howlong>", methods=['POST'])
def smallarmUp(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   speed = -(speed)
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m6.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m6.setMotorSpeed(0)
   return "ok"

@app.route("/smallarm/down/<int:speed>/<int:howlong>", methods=['POST'])
def smallarmDown(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m6.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m6.setMotorSpeed(0)
   return "ok"

@app.route("/bucket/up/<int:speed>/<int:howlong>", methods=['POST'])
def bucketUp(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m5.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m5.setMotorSpeed(0)
   return "ok"

@app.route("/bucket/down/<int:speed>/<int:howlong>", methods=['POST'])
def bucketDown(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   speed = -(speed)
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m5.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m5.setMotorSpeed(0)
   return "ok"

@app.route("/move/forward/<int:speed>/<int:howlong>", methods=['POST'])
def moveForward(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m3.setMotorSpeed(int(-speed))
   m4.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m3.setMotorSpeed(0)
   m4.setMotorSpeed(0)
   return "ok"

@app.route("/move/backward/<int:speed>/<int:howlong>", methods=['POST'])
def moveBackward(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m3.setMotorSpeed(int(speed))
   m4.setMotorSpeed(int(-speed))
   time.sleep(howlong/1000)
   m3.setMotorSpeed(0)
   m4.setMotorSpeed(0)
   return "ok"

@app.route("/turn/left/<int:speed>/<int:howlong>", methods=['POST'])
def turnLeft(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m3.setMotorSpeed(int(speed))
   m4.setMotorSpeed(int(speed))
   time.sleep(howlong/1000)
   m3.setMotorSpeed(0)
   m4.setMotorSpeed(0)
   return "ok"

@app.route("/turn/right/<int:speed>/<int:howlong>", methods=['POST'])
def turnRight(speed, howlong):
   if speed > 255:
     speed = 255
   if speed < 0:
     speed = 0
   print "Speed: %d" % speed
   print "Time: %d" % howlong
   m3.setMotorSpeed(int(-speed))
   m4.setMotorSpeed(int(-speed))
   time.sleep(howlong/1000)
   m3.setMotorSpeed(0)
   m4.setMotorSpeed(0)
   return "ok"

if __name__ == "__main__":
   app.run(host='0.0.0.0')

m1.off()
m2.off()
m3.off()
m4.off()
m5.off()
m6.off()

