import time
import smbus
import datetime
import RPi.GPIO as GPIO
#temperature variables:
PS = 1
CSB = 1
i2c_ch = 1

i2c_address = 0x77
reg_reset = 0x1E
reg_ADC_start = 0x48
reg_ADC = 0x00
reg_read = 0xA0
reg_k4 = 0xA2
reg_k3 = 0xA4
reg_k2 = 0xA6
reg_k1 = 0xA8
reg_k0 = 0xAA
bus = smbus.SMBus(i2c_ch)
bus.write_byte(i2c_address, reg_reset)
time.sleep(.1)


def readTemp():
    bus.write_byte_data(i2c_address, reg_k0, 0)
    k_0 = bus.read_i2c_block_data(i2c_address, reg_k0)
    k_0_1 = k_0[0]
    k_0_2 = k_0[1]
    k_0_1_bit = bin(k_0_1)[2:].zfill(8)
    k_0_2_bit = bin(k_0_2)[2:].zfill(8)
    k_0_tot = k_0_1_bit + k_0_2_bit
    k_0_final = int(k_0_tot, 2)
    bus.write_byte_data(i2c_address, reg_k1, 0)
    k_1 = bus.read_i2c_block_data(i2c_address, reg_k1)
    k_1_1 = k_1[0]
    k_1_2 = k_1[1]
    k_1_1_bit = bin(k_1_1)[2:].zfill(8)
    k_1_2_bit = bin(k_1_2)[2:].zfill(8)
    k_1_tot = k_1_1_bit + k_1_2_bit
    k_1_final = int(k_1_tot, 2)
    bus.write_byte_data(i2c_address, reg_k2, 0)
    k_2 = bus.read_i2c_block_data(i2c_address, reg_k2)
    k_2_1 = k_2[0]
    k_2_2 = k_2[1]
    k_2_1_bit = bin(k_2_1)[2:].zfill(8)
    k_2_2_bit = bin(k_2_2)[2:].zfill(8)
    k_2_tot = k_2_1_bit + k_2_2_bit
    k_2_final = int(k_2_tot, 2)
    bus.write_byte_data(i2c_address, reg_k3, 0)
    k_3 = bus.read_i2c_block_data(i2c_address, reg_k3)
    k_3_1 = k_3[0]
    k_3_2 = k_3[1]
    k_3_1_bit = bin(k_3_1)[2:].zfill(8)
    k_3_2_bit = bin(k_3_2)[2:].zfill(8)
    k_3_tot = k_3_1_bit + k_3_2_bit
    k_3_final = int(k_3_tot, 2)
    bus.write_byte_data(i2c_address, reg_k4, 0)
    k_4 = bus.read_i2c_block_data(i2c_address, reg_k4)
    k_4_1 = k_4[0]
    k_4_2 = k_4[1]
    k_4_1_bit = bin(k_4_1)[2:].zfill(8)
    k_4_2_bit = bin(k_4_2)[2:].zfill(8)
    k_4_tot = k_4_1_bit + k_4_2_bit
    k_4_final = int(k_4_tot, 2)
    
    bus.write_byte(i2c_address, reg_ADC_start)
    time.sleep(.2)
    ADC = bus.read_i2c_block_data(i2c_address, reg_ADC)
    ADC_1 = ADC[0]
    ADC_2 = ADC[1]
    ADC_3 = ADC[2]
    ADC_1_bit = bin(ADC_1)[2:].zfill(8)
    ADC_2_bit = bin(ADC_2)[2:].zfill(8)
    ADC_3_bit = bin(ADC_3)[2:].zfill(8)
    ADC_tot = ADC_1_bit + ADC_2_bit + ADC_3_bit
    ADC_final = int(ADC_tot, 2) / 256
    temp_c = -2 * k_4_final * (10**(-21)) * (ADC_final**4)   
    temp_c = temp_c + 4 * k_3_final *(10**(-16)) * (ADC_final**3)
    temp_c = temp_c + -2 * k_2_final * (10**(-11)) * (ADC_final**2)
    temp_c = temp_c + 1 * k_1_final * (10**(-6)) * (ADC_final)
    temp_c = temp_c + -1.5 * k_0_final * (10**(-2))
    return temp_c
def heating_on():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18,GPIO.OUT)
    print ("LED on")
    GPIO.output(18,GPIO.HIGH)
def heating_off():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18,GPIO.OUT)
    GPIO.output(18,GPIO.LOW)


class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value
        print("P:")
        print(self.P_value)
        print("I:")
        print(self.I_value)
        print("D:")
        print(self.D_value)
        print("PID")
        print(PID)
        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

T = 20
p = PID(1, 10, 1)
p.setPoint(50.0)
while T != p.getPoint():
    pid = p.update(T)
    print(T)
    time.sleep(1)
    T += 1
