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

serial_pattern = re.compile('[a-z0-9]+:[0-9]+', re.IGNORECASE)

def read_serial(hardware):
    """
    Read whatever is coming from the reciever.
    hardware    the serial port to read from
    Returns the recieved data package as a dict of param_id->value pairs    
    """
    #hardware.write(bytes(x, 'utf-8'))
    recieved_data = hardware.readline()
    if recieved_data:
        print(f"RCV: {recieved_data}")
        recieved_data = recieved_data.decode('ascii') # decode from byte string 

        # Assuming data is coming in the format "ad:12837,bc:123,c2:8762"
        if serial_pattern.match(recieved_data):
            res = dict()
            vals = recieved_data.split(',')
            try:
                for val in vals:
                    param_id, value = val.split(':')
                    res[param_id] = int(value)
                return res            
            except ValueError as e:
                print(f"!!! {recieved_data}")
                return None

        else:
            print(f"> {recieved_data}")
            return None


if __name__=="__main__":
    import argparse
    from pythonosc import osc_message_builder
    from pythonosc import udp_client

    parser = argparse.ArgumentParser(description='Utility to convert incoming serial data from an ESP32 reciever to OSC.')

    parser.add_argument('--list_serial', action='store_true', default=False, help='List all available serial ports.')


    parser.add_argument('--serial_port', type=str, help='Choose serial port the ESP32 reciever is connected to, by default uses the first port found with --list_serial')
    parser.add_argument('--serial_baud', type=int, default=115200, help='Choose serial port baud rate. Uses 115200 by default.')
    parser.add_argument('--osc_dest', type=str, default='localhost:57120', help='Destination address and port to send OSC data to. By default sends to localhost:57120')
    parser.add_argument('--osc_address', type=str, default='/esp32', help='The OSC address that will be recieved by the destination software. Uses /esp32 by default.')
        
    args = parser.parse_args()

    if args.list_serial:
        print("Available Serial Ports")
        print(serial_ports())
    else:
        OSC_DESTINATION_IP, OSC_DESTINATION_PORT = args.osc_dest.split(':')
        OSC_DESTINATION_PORT = int(OSC_DESTINATION_PORT)
        SERIAL_PORT = args.serial_port
        SERIAL_BAUD = args.serial_baud
        if SERIAL_PORT is None:
            ports = serial_ports()
            if not ports:
                raise ConnectionError("No Serial Ports can be Found")
            SERIAL_PORT = ports[0]

        # Create OSC client & connect to Serial port
        osc_client = udp_client.SimpleUDPClient(OSC_DESTINATION_IP, OSC_DESTINATION_PORT)        
        hardware = serial.Serial(port=SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=.1)

        while True:
            time.sleep(0.005) # serial read rate...
            datablock=read_serial(hardware)

            if datablock is not None:
                builder = osc_message_builder.OscMessageBuilder(address="/esp32")
                for param_id, value in datablock.items():
                    builder.add_arg(value)
                msg = builder.build()
                #Send out to OSC client
                osc_client.send(msg)
                print(f"{datablock}") # give some feedback
