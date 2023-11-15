#############################################################
#   Convert incoming Serial from an Arduino sketch to OSC
#
#   Jonathan Reus 2023
#
#   Dependencies: pyserial, python-osc
#
#   TODO: set up argparse for command line arguments
#   TODO: implement OSC comms
#   TODO: make some kind of gui for all this
#    
#   Run with: python Serial2OSC.py -p /reciever/serial/port 
#
############################################################

import serial
import sys
import time
import re
import glob

#SERIALPORT='COM4' # Windows looks like this
SERIALPORT='/dev/ttyUSB1' # Ubuntu Linux
#SERIALBAUD=115200
SERIALBAUD=230400

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result




class DataPackage:
    """
    Data Package
    Needs to be defined specifically to match the type of data coming in from the reciever.
    """
    params = list()
    timestamp = None

    def __init__(self, ts, *params):
        if ts is None:
            self.timestamp = time.time()
        else:
            self.timestamp = ts
        for idx,param in enumerate(params):
            if idx == 0:
                # Add any special parameter specific processing here, otherwise assumes unsigned 12-bit integers
                pass 
            self.params.append(param)


###########################################################################
################# Serial Read Functions
###########################################################################
def read_int(buffer, start):
    strval=""
    idx = start
    while (idx < len(buffer)) and (buffer[idx] not in "abcdefghijklmnop"): 
        strval += buffer[idx]
        idx += 1
    return int(strval),idx

def write_read(x, hardware):
    """
    Write parameter x to the serial buffer.
    Then read whatever is coming from the reciever.

    Returns the recieved data package
    
    """
    #hardware.write(bytes(x, 'utf-8'))
    pattern = re.compile('[a-z]:', re.IGNORECASE)
    time.sleep(0.05)
    recieved_data = hardware.readline()
    if recieved_data:
        print(f"RCV: {recieved_data}")
        recieved_data = recieved_data.decode('ascii') # decode from byte string 

        # Assuming data is coming in the format "a:12837" where a is the param id and 12837 is the value
        # if pattern.match(recieved_data[:2]):
        #     param_id = recieved_data[0]
        #     # Depending on id, do special parsing...
        #     # if param_id is 'a': # for example...
        #     #     value = float(recieved_data[2:])
            
        #     value = int(recieved_data[2:])
        #     return param_id, value


        # Assuming data is coming in the format "a:12837,b:123,c:8762"
        if pattern.match(recieved_data[:2]):
            res = dict()
            vals = recieved_data.split(',')
            try:
                for val in vals:
                    param_id = val[0]
                    value = int(val[2:])
                    res[param_id] = value
                return res            
            except ValueError as e:
                print(f"!!! {recieved_data}")
                return None

        else:
            print(f"> {recieved_data}")
            return None


if __name__=="__main__":
    
    from pythonosc import osc_message_builder
    from pythonosc import udp_client

    OSC_DESTINATION_IP = 'localhost'
    OSC_DESTINATION_PORT = 57120 # default local port for sclang
    
    osc_client = udp_client.SimpleUDPClient(OSC_DESTINATION_IP, OSC_DESTINATION_PORT)

    print("List Serial Ports")
    print(serial_ports())
    
    hardware = serial.Serial(port=SERIALPORT, baudrate=SERIALBAUD, timeout=.1)

    while True:
        datablock=write_read(0, hardware)

        if datablock is not None:
            builder = osc_message_builder.OscMessageBuilder(address="/esp32")
            
            for param_id, value in datablock.items():
                builder.add_arg(value)

            msg = builder.build()

            #Send out to OSC client
            osc_client.send(msg)
            
            print(f"{datablock}")
