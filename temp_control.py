#From dual_unipolar_temp_controller by Shreyas for Arduino
#Translated to python for Labjack by Murali - Summer 2018
#Updated by Shira - Dec 2018

from labjack import ljm
import time as time
import numpy as np
import threading
import datetime
import sys
import zmq

handle = ljm.openS("T7", "USB", "ANY")

## Useful classes, functions, etc.

# measured values of resistors in each resistive divider
R1 = 9982
R2 = 9961
R3 = 9962
R4 = 9981

class Channel:
    def __init__(self,ref,therm,output,resis):
        self.ref = ref
        self.therm = therm
        self.output = output
        self.resis = resis

# which LabJack input/outputs correspond to which channels
chan1 = Channel("AIN7","AIN6","TDAC1",R1)
chan2 = Channel("AIN5","AIN4","TDAC0",R2)
chan3 = Channel("AIN3","AIN2","DAC1",R3)
chan4 = Channel("AIN1","AIN0","DAC0",R4)

# constants for temperature conversion for Thorlabs 10k thermistors (from https://www.thorlabs.com/_sd.cfm?fileName=4813-S01.pdf&partNumber=TH10K)
a = 3.354017e-3
b = 2.5617244e-4
c = 2.1400943e-6
d = -7.2405219e-8

def get_temp(channel):
    """Returns temperature in Degrees Celsius"""
    #Calculates thermistor resistance
    vain_therm = ljm.eReadName(handle, channel.therm) #Voltage divider voltage, in V
    vain_ref = ljm.eReadName(handle, channel.ref) #Total positive to negative voltage, in V
    resistance = float(vain_therm/(vain_ref-vain_therm))*channel.resis #Thermistor resistance calculation
    logarith = np.log(resistance/10000.0) #Logarithm required
    T = 1.0/(a + b * logarith + c * (logarith ** 2) + d * (logarith ** 3)) #Temperature calulcation
    return (T - 273.15)


A = -1.6443767e1
B = 6.1080608e3
C = -4.4141671e5
D = 2.4159818e7

def thermistor_resistance(temp):
    #Code for thermistor resistance calculation. Equation available: https://www.thorlabs.com/_sd.cfm?fileName=4813-S01.pdf&partNumber=TH10K
    tempC = temp + 273.15
    Rth = (10000.0)*np.exp(A + B/tempC + C/(tempC ** 2) + D/(tempC ** 3))
    return Rth #Thermistor resistance

def set_voltage(channel,rth):
    #Voltage calculation
    V = ljm.eReadName(handle, channel.ref) * (rth / (rth+channel.resis))
    return V

## ZMQ Publishing
class zmq_pub_dict:
    """Publishes a python dictionary on a port with a given topic."""

    def __init__(self, port=5551, topic='temp_control'):
        zmq_context = zmq.Context()
        self.topic = topic
        self.pub_socket = zmq_context.socket(zmq.PUB)

        self.pub_socket.bind("tcp://*:%s" % port)
        print('Broadcasting on port {0} with topic {1}'.format(port,
                                                               topic))

    def send(self, data_dict):
        timestamp = time.time()
        send_string = "%s %f %s" % (self.topic, timestamp, repr(data_dict))
        print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()

    def publish_data(self,data):
        try:
            err1,corr1,err2,corr2,err3,corr3,err4,corr4 = data
            data_dict = {'Error1 (C)': err1, 'Correction1 (V)': corr1,'Error2 (C)': err2, 'Correction2 (V)': corr2,'Error3 (C)': err3, 'Correction3 (V)': corr3,'Error4 (C)': err4, 'Correction4 (V)': corr4 }
            dt = str(time.time())
            self.send(data_dict)
        except:
            print('error')

## Threading
kbd_input = ''
new_input = True

def commandListener():
    global kbd_input, new_input
    kbd_input = raw_input()
    new_input = True

## Servo Code

N_ACCUM = 100
ZEROV = 0.0
GATE_VOLT_MIN = 1.5
GATE_VOLT_MAX = 3.0

class Servo:
    def __init__(self,channel,enable=False,set = 23.0, gain_p = 2.0, gain_i = 0.1, gain_d = 1000):
        self.channel = channel
        self.enable = enable
        self.set = set
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.gain_d = gain_d
        self.small_accumulator = 0
        self.accumulator = 0
        self.prev_sig = 0
        self.n_accum = 0
        ljm.eWriteName(handle, self.channel.output, ZEROV)
        self.rth = thermistor_resistance(set) #what the thermistor resistance should be at setpoint temp

    def reset(self):
        ljm.eWriteName(handle, self.channel.output, ZEROV)


    def getErrorSignal(self):

        error_signal_new =  ljm.eReadName(handle, self.channel.therm) - ljm.eReadName(handle, self.channel.ref) * (self.rth / (self.rth+self.channel.resis))

        self.n_accum +=1
        self.small_accumulator += error_signal_new*dt
        if (self.n_accum > N_ACCUM):
            self.accumulator += self.small_accumulator
            self.n_accum = 0
            self.small_accumulator = 0.0

        return error_signal_new

    def getCorrectionSignal(self,new_error):

        prop_term = alpha_avg*self.prev_sig + (1.0-alpha_avg)*(new_error*self.gain_p)
        self.prev_sig = prop_term
        der_term = (new_error - self.prev_sig)/dt/self.gain_d
        der_term = 0.995*der_term + 0.005*(der_term*self.gain_p)
        int_term = self.accumulator*self.gain_i*self.gain_p

        corr = int_term + prop_term #NOT USING DERIVATIVE TERM RIGHT NOW

        if(corr>GATE_VOLT_MAX):
            corr = GATE_VOLT_MAX
            self.accumulator = GATE_VOLT_MAX/(self.gain_i*self.gain_p)


        if(corr<GATE_VOLT_MIN):
            corr = GATE_VOLT_MIN
            self.accumulator = GATE_VOLT_MIN/(self.gain_i*self.gain_p)

        return corr

    def writeCorrection(self,corr):
        if(self.enable):
            ljm.eWriteName(handle, self.channel.output, corr)
        else:
            ljm.eWriteName(handle, self.channel.output, ZEROV)

    def turnOn(self):
        self.enable = True
        print("Servo on")

    def turnOff(self):
        self.enable = False
        self.reset()
        print("Servo off")

    def gainP(self,new_gain):
        self.gain_p = new_gain
        print("Proportional gain updated to %.2f"%new_gain)

    def gainI(self,new_gain):
        self.gain_i = new_gain
        print("Intergral gain updated to %.2f"%new_gain)

    def setTemp(self,new_temp):
        self.set = new_temp
        self.rth = thermistor_resistance(new_temp)
        self.small_accumulator = 0
        self.accumulator = 0
        self.prev_sig = 0
        print("Setpoint temperature updated to %.1f"%new_temp)


publish = False

def publishOn():
    global publish
    publish = True
    print("Publishing...")

def publishOff():
    global publish
    publish = False
    print("Publishing off")

## Loop

# Set temperatures:
S1 = 22.0
S2 = 22.0
S3 = 23.0
S4 = 22.0

# Default/initial servo parameters
servo1 = Servo(chan1,set = S1,gain_p = 2.0,gain_i = 0.05)
servo2 = Servo(chan2,set = S2,gain_p = 4.0,gain_i = 0.1)
servo3 = Servo(chan3,set = S3,gain_p = 2.0,gain_i = 0.1)
servo4 = Servo(chan4,set = S4,gain_p = 2.0,gain_i = 0.05)

counter = 0
curr_time = time.time()

#low-pass filtering constant
alpha_avg = 0.99

print("Loop running...")

publisher = zmq_pub_dict(5551,'temp_control')
publish = False

loop_running = True

while(loop_running):
    try:
        counter +=1

        prev_time = curr_time
        curr_time = time.time()
        dt = curr_time - prev_time

        err1 = servo1.getErrorSignal()
        err2 = servo2.getErrorSignal()
        err3 = servo3.getErrorSignal()
        err4 = servo4.getErrorSignal()

        corr1,corr2,corr3,corr4 = 0,0,0,0

        if(servo1.enable): corr1 = servo1.getCorrectionSignal(err1)
        if(servo2.enable): corr2 = servo2.getCorrectionSignal(err2)
        if(servo3.enable): corr3 = servo3.getCorrectionSignal(err3)
        if(servo4.enable): corr4 = servo4.getCorrectionSignal(err4)

        servo1.writeCorrection(corr1)
        servo2.writeCorrection(corr2)
        servo3.writeCorrection(corr3)
        servo4.writeCorrection(corr4)

        if(publish and counter%100==0):
            try:
                temp1, temp2, temp3, temp4 = 0,0,0,0
                if(servo1.enable):
                    temp1 = get_temp(chan1) - servo1.set
                if(servo2.enable):
                    temp2 = get_temp(chan2) - servo2.set
                if(servo3.enable):
                    temp3 = get_temp(chan3) - servo3.set
                if(servo4.enable):
                    temp4 = get_temp(chan4) - servo4.set

                data = (temp1,corr1,temp2,corr2,temp3,corr3,temp4,corr4)
                publisher.publish_data(data)
            except:
                print("Publishing error")

        if new_input:
            try:
                exec(kbd_input)
            except:
                print("Invalid input")
            new_input = False
            listener = threading.Thread(target=commandListener)
        listener.start()

    except KeyboardInterrupt:
        print("Loop exited")
        servo1.reset()
        servo2.reset()
        servo3.reset()
        servo4.reset()
        publisher.close()
        loop_running = False

    except:
        pass

publisher.close()
