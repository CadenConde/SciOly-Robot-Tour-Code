from machine import Pin, I2C
import time
import utime
import math
#from imu import MPU6050

class PCA9685:
    # Registers/etc.
    __SUBADR1            = 0x02
    __SUBADR2            = 0x03
    __SUBADR3            = 0x04
    __MODE1              = 0x00
    __PRESCALE           = 0xFE
    __LED0_ON_L          = 0x06
    __LED0_ON_H          = 0x07
    __LED0_OFF_L         = 0x08
    __LED0_OFF_H         = 0x09
    __ALLLED_ON_L        = 0xFA
    __ALLLED_ON_H        = 0xFB
    __ALLLED_OFF_L       = 0xFC
    __ALLLED_OFF_H       = 0xFD

    def __init__(self, address=0x40, debug=False):
        LED = machine.Pin("LED", machine.Pin.OUT)
        LED.on
        self.i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq=100000)
        self.address = address
        self.debug = debug
        if (self.debug):
            print("Reseting PCA9685") 
        self.write(self.__MODE1, 0x00)
	
    def write(self, cmd, value):
        "Writes an 8-bit value to the specified register/address"
        self.i2c.writeto_mem(int(self.address), int(cmd), bytes([int(value)]))
        if (self.debug):
            print("I2C: Write 0x%02X to register 0x%02X" % (value, cmd))
	  
    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        rdate = self.i2c.readfrom_mem(int(self.address), int(reg), 1)
        if (self.debug):
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, int(reg), rdate[0]))
        return rdate[0]
	
    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if (self.debug):
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if (self.debug):
            print("Final pre-scale: %d" % prescale)

        oldmode = self.read(self.__MODE1)
        #print("oldmode = 0x%02X" %oldmode)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
        self.write(self.__LED0_ON_H+4*channel, on >> 8)
        self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
        self.write(self.__LED0_OFF_H+4*channel, off >> 8)
        if (self.debug):
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel,on,off))
	  
    def setServoPulse(self, channel, pulse):
        pulse = pulse * (4095 / 100)
        self.setPWM(channel, 0, int(pulse))
    
    def setLevel(self, channel, value):
        if (value == 1):
              self.setPWM(channel, 0, 4095)
        else:
              self.setPWM(channel, 0, 0)

class MotorDriver():
    def __init__(self, debug=False):
        
        #encoder logic----------------------------------------
        self.pinA = Pin(10, Pin.IN)
        self.pinB = Pin(11, Pin.IN)
        self.pinC = Pin(6, Pin.IN)
        self.pinD = Pin(7, Pin.IN)
        
        self.countA = 0; #motor 1 clicks
        self.countB = 0; #motor 2 clicks

        def callbackA(pin):
            self.countA += 1
            print("ping!")
            
        def callbackB(pin):
            self.countB += 1
            
        self.pinA.irq(trigger=Pin.IRQ_FALLING, handler=callbackA)
        self.pinA.irq(trigger=Pin.IRQ_RISING, handler=callbackA)
        self.pinB.irq(trigger=Pin.IRQ_FALLING, handler=callbackA)
        self.pinB.irq(trigger=Pin.IRQ_RISING, handler=callbackA)
        
        self.pinC.irq(trigger=Pin.IRQ_FALLING, handler=callbackB)
        self.pinC.irq(trigger=Pin.IRQ_RISING, handler=callbackB)
        self.pinD.irq(trigger=Pin.IRQ_FALLING, handler=callbackB)
        self.pinD.irq(trigger=Pin.IRQ_RISING, handler=callbackB)
        #--------------------------------------------------------
        
        self.debug = debug
        self.pwm = PCA9685()
        self.LED = machine.Pin("LED", machine.Pin.OUT)
        #self.imu = MPU6050(self.pwm.i2c)
        self.pwm.setPWMFreq(50)       
        self.MotorPin = ['MA', 0,1,2, 'MB',3,4,5, 'MC',6,7,8, 'MD',9,10,11]
        self.MotorDir = ['forward', 0,1, 'backward',1,0]
        
    def MotorRun(self, motor, mdir, speed):
        if speed > 100:
            return
        
        mPin = self.MotorPin.index(motor)
        mDir = self.MotorDir.index(mdir)
        
        if (self.debug):
            print("set PWM PIN %d, speed %d" %(self.MotorPin[mPin+1], speed))
            print("set pin A %d , dir %d" %(self.MotorPin[mPin+2], self.MotorDir[mDir+1]))
            print("set pin b %d , dir %d" %(self.MotorPin[mPin+3], self.MotorDir[mDir+2]))

        self.pwm.setServoPulse(self.MotorPin[mPin+1], speed)        
        self.pwm.setLevel(self.MotorPin[mPin+2], self.MotorDir[mDir+1])
        self.pwm.setLevel(self.MotorPin[mPin+3], self.MotorDir[mDir+2])

    def MotorStop(self, motor):
        mPin = self.MotorPin.index(motor)
        
        self.pwm.setServoPulse(self.MotorPin[mPin+1], 0)
        self.pwm.setLevel(self.MotorPin[mPin+2], 0)
        self.pwm.setLevel(self.MotorPin[mPin+3], 0)
        
    def EnterTrack(self):
        distance = 700
        distanceB = distance - 20
        self.countA = 0
        self.countB = 0
        stopA = False
        stopB = False
        self.MotorRun('MB', 'backward', 80)
        self.MotorRun('MA', 'backward', 80)
        while True:
            if(self.countB>distanceB and not stopB): #stop
                self.MotorStop('MB')
                stopB = True
            elif(self.countB>(distanceB-50)): #decel 2
                self.MotorRun('MB', 'backward', 20)
            elif(self.countB>(distanceB-100)): #decel 1
                self.MotorRun('MB', 'backward', 40)
                
            if(self.countA>distance and not stopA):
                self.MotorStop('MA')
                stopA = True
            elif(self.countA>(distance-50)):
                self.MotorRun('MA', 'backward', 20)
            elif(self.countA>(distance-100)):
                self.MotorRun('MA', 'backward', 40)
            
            if(stopA and stopB): #stop
                self.MotorStop('MA')
                self.MotorStop('MB')
                print(str(self.countA) + " " + str(self.countB))
                break
        
    def Forward(self):
        distance = 1600
        distanceB = distance+107
        self.countA = 0
        self.countB = 0
        stopA = False
        stopB = False
        self.MotorRun('MB', 'backward', 60)
        self.MotorRun('MA', 'backward', 60)
        while True:
            if(self.countB>distanceB and not stopB): #stop
                self.MotorStop('MB')
                stopB = True
            elif(self.countB>(distanceB-30)): #decel 2
                self.MotorRun('MB', 'backward', 20)
            elif(self.countB>(distanceB-60)): #decel 1
                self.MotorRun('MB', 'backward', 40)
                
            if(self.countA>distance and not stopA):
                self.MotorStop('MA')
                stopA = True
            elif(self.countA>(distance-30)):
                self.MotorRun('MA', 'backward', 20)
            elif(self.countA>(distance-6-0)):
                self.MotorRun('MA', 'backward', 40)
            
            if(stopA and stopB): #stop
                self.MotorStop('MA')
                self.MotorStop('MB')
                print(str(self.countA) + " " + str(self.countB))
                break

    def TurnLeft(self):
        distance = 0
        distanceB = distance + 30
        self.countA = 0
        self.countB = 0
        stopA = False
        stopB = False
        self.MotorRun('MB', 'forward', 40)
        self.MotorRun('MA', 'backward', 40)
        while True:
            if(self.countB>distanceB and not stopB): #stop
                self.MotorStop('MB')
                stopB = True
            elif(self.countB>(distanceB-25)): #decel 2
                self.MotorRun('MB', 'forward', 10)
            elif(self.countB>(distanceB-50)): #decel 1
                self.MotorRun('MB', 'forward', 20)
                
            if(self.countA>distance and not stopA):
                self.MotorStop('MA')
                stopA = True
            elif(self.countA>(distance-25)):
                self.MotorRun('MA', 'backward', 10)
            elif(self.countA>(distance-50)):
                self.MotorRun('MA', 'backward', 20)
            
            if(stopA and stopB): #stop
                self.MotorStop('MA')
                self.MotorStop('MB')
                print(str(self.countA) + " " + str(self.countB))
                break
         
    def TurnRight(self):
        distance = 420
        distanceB = distance + 30
        self.countA = 0
        self.countB = 0
        stopA = False
        stopB = False
        self.MotorRun('MB', 'backward', 40)
        self.MotorRun('MA', 'forward', 40)
        while True:
            if(self.countB>distanceB and not stopB): #stop
                self.MotorStop('MB')
                stopB = True
            elif(self.countB>(distanceB-25)): #decel 2
                self.MotorRun('MB', 'backward', 10)
            elif(self.countB>(distanceB-50)): #decel 1
                self.MotorRun('MB', 'backward', 20)
                
            if(self.countA>distance and not stopA):
                self.MotorStop('MA')
                stopA = True
            elif(self.countA>(distance-25)):
                self.MotorRun('MA', 'forward', 10)
            elif(self.countA>(distance-50)):
                self.MotorRun('MA', 'forward', 20)
            
            if(stopA and stopB): #stop
                self.MotorStop('MA')
                self.MotorStop('MB')
                print(str(self.countA) + " " + str(self.countB))
                break
    
if __name__ == '__main__':
    led_pin = Pin("LED", Pin.OUT)
    led_pin.on()
    #ultrasonic sensor code------
    trigger = Pin(3, Pin.OUT)
    echo = Pin(2, Pin.IN)
    def ultra():
        trigger.low()
        utime.sleep_us(2)
        trigger.high()
        utime.sleep_us(5)
        trigger.low()
        while echo.value() == 0:
            signaloff = utime.ticks_us()
        while echo.value() == 1:
            signalon = utime.ticks_us()
        timepassed = signalon - signaloff
        distance = (timepassed * 0.0343) / 2
        print("The distance from object is ",distance,"cm")
    #-------------------------
    
    m = MotorDriver()
    
    
    #Pathing code-------------------
    try:
        path = ["r","l","r","l","x","x"]
        delay = 2
        for i in path:
            if i == "e":
                m.EnterTrack()
                time.sleep(delay)
            elif i == "f":
                m.Forward()
                time.sleep(delay)
            elif i == "r":
                m.TurnRight()
                time.sleep(delay)
            elif i == "l":
                m.TurnLeft()
                time.sleep(delay)
            else:
                continue
    #-----------------------------------
    except KeyboardInterrupt:
        exit()
