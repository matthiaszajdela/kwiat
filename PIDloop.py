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

#PID variables:
t_set = 50
p_fact = 1.0
i_fact = -0.01
i_total = 0.0
d_fact = -5.0
last_error = None
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
    GPIO.output(18,GPIO.HIGH)
def heating_off():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18,GPIO.OUT)
    GPIO.output(18,GPIO.LOW)
def loop():
    while True:
      error = t_set - readTemp()
      if last_error is None:
        last_error = error
      i_total += error
      if (error * p_fact + (error - last_error) * d_fact + i_total * i_fact) > 0.0:
        heating_on()
      else:
        heating_off()
      last_error = error
      time.sleep(300.0)
    
    

