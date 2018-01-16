import time
from darkwater_640 import dw_Controller, dw_Motor

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
