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

import sys
import os
import time
import re
import glob
import multiprocessing
import traceback
import logging
import serial

serial_pattern = re.compile('[a-z0-9]+:[0-9]+', re.IGNORECASE)

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

def proc_osc_server(pipe_to_main: multiprocessing.Pipe, msg_queue: multiprocessing.Queue, server_addr: tuple[str, int]) -> None:
    """
    Meant to be run in its own process. 
    Sets up an OSC server and pipes incoming messages to a queue in the main process.

    pipe_to_main    Interprocess communication pipe to main process
    msg_queue       Interprocess message queue, gets filled with incoming messages to be consumed in main process
    server_addr     (ip, port) of the OSC server
    """

    from pythonosc.dispatcher import Dispatcher
    from pythonosc.osc_server import BlockingOSCUDPServer

    # There are a few options for server request handling. Here use the most basic BlockingOSCUDPServer
    # See: https://python-osc.readthedocs.io/en/latest/server.html

    def default_handler(address: str, *args) -> None:
        # Pipe everything to the main process
        msg = { 'address': address, 'args': args }
        msg_queue.put(msg)

    state = type('OSCServerState', (), {})()
    state.main_pipe = pipe_to_main

    # Set up OSC Dispatcher and server
    print(f"Starting OSC Server in process {os.getpid()}")
    state.dispatcher = Dispatcher()
    #state.dispatcher.map("/misc/*", misc_handler)
    state.dispatcher.set_default_handler(default_handler)
    state.addr = server_addr
    state.server = BlockingOSCUDPServer((state.addr[0], state.addr[1]), state.dispatcher)

    try:
        print(f"Listening for incoming OSC on: {state.addr}")
        state.server.serve_forever()  # Blocks forever
    except KeyboardInterrupt as e:
        print("WARNING: Got KeyboardInterrupt in OSC Server Process")
    except Exception as e:
        print(f"ERROR: Encountered unexpected error in OSC Server Process: {e.__class__.__name__}: {str(e)}")
        traceback.format_exc()
        # TODO: Here I should let the main proc know something went wrong...
        #   or rather, the main proc should recognize the text proc failed
        #   and try to recover, or do something else...

    print("Exiting OSC Server Process")


def write_read_serial(hardware: serial.Serial, osc_msg_queue: multiprocessing.Queue) -> dict[str,int]:
    """
    Write whatever is in the send queue to the reciever.
    Read whatever is coming from the reciever & send to local software via OSC.

    hardware    the open serial port

    Returns any recieved data package as a dict of param_id->value pairs    
    """
    res = None

    # 1) Process all messages currently in the OSC queue
    if not osc_msg_queue.empty():
        for i in range(osc_msg_queue.qsize()):
            oscmsg = osc_msg_queue.get()
            # write all the data to Serial
            print(f"Write to serial: {oscmsg}")
            node,actuator,target_value = oscmsg['args']
            serial_str = f"{node},{actuator},{target_value}"
            hardware.write(bytes(serial_str, 'utf-8'))
    
    # 2) Read any incoming messages from the coordinator
    raw_recieved_data = hardware.readline()
    if raw_recieved_data:
        #print(f"RCV: {recieved_data}")
        recieved_data = raw_recieved_data.decode('ascii') # decode from byte string 

        # Assuming data is coming in the format "ad:12837,bc:123,c2:8762"
        if serial_pattern.match(recieved_data):
            res = dict()
            vals = recieved_data.split(',')
            try:
                for val in vals:
                    param_id, value = val.split(':')
                    res[param_id] = int(value)            
            except ValueError as e:
                print(f":! {raw_recieved_data}")
                res=None
        else:
            res=None
            if raw_recieved_data != b'\r\n':
                print(f": {raw_recieved_data}")
            else:
                print(":")
  
    return res


def run_main_loop(osc_client, serial_conn: serial.Serial, osc_msg_queue: multiprocessing.Queue) -> None:
    """
    Main application loop. 
    
    """
    print("Begin serial read loop")
    while True:
        time.sleep(0.005) # serial read rate...
        datablock=write_read_serial(hardware=serial_conn, osc_msg_queue=osc_msg_queue)

        if datablock is not None:
            builder = osc_message_builder.OscMessageBuilder(address="/esp32")
            for param_id, value in datablock.items():
                builder.add_arg(value)
            msg = builder.build()
            #Send out to OSC client
            osc_client.send(msg)
            print(f"{datablock}")



if __name__=="__main__":
    import argparse
    from pythonosc import osc_message_builder
    from pythonosc import udp_client
    
    parser = argparse.ArgumentParser(description='Utility to convert incoming serial data from an ESP32 reciever to OSC.')

    parser.add_argument('--list_ports', action='store_true', default=False, help='List all available serial ports.')

    parser.add_argument('--serial_port', type=str, help='Choose serial port the ESPNow coordinator is connected to, by default uses the first port found with --list_serial')
    parser.add_argument('--serial_baud', type=int, default=115200, help='Choose serial port baud rate, must match baud rate in the coordinator firmware. Uses 115200 by default.')
    
    # Outgoing OSC parameters
    parser.add_argument('--osc_addr', type=str, default='localhost:57120', help='Destination address and port to send OSC data to. By default sends to localhost:57120')
    parser.add_argument('--osc_path', type=str, default='/esp32', help='The OSC path of messages sent by this app. Uses /esp32 by default.')
        
    # Incoming OSC parameters
    parser.add_argument('--osc_server_addr', type=str, default='localhost:1337', help='OSC ip/port of this app for recieving OSC messages from client software. By default listens on localhost:1337')


    args = parser.parse_args()

    if args.list_ports:
        print("Available Serial Ports")
        print(serial_ports())
    else:
        OSC_DESTINATION_IP, OSC_DESTINATION_PORT = args.osc_addr.split(':')
        OSC_DESTINATION_PORT = int(OSC_DESTINATION_PORT)
        OSC_SERVER_IP, OSC_SERVER_PORT = args.osc_server_addr.split(':')
        OSC_SERVER_PORT = int(OSC_SERVER_PORT)
        SERIAL_PORT = args.serial_port
        SERIAL_BAUD = args.serial_baud
        if SERIAL_PORT is None:
            ports = serial_ports()
            if not ports:
                raise ConnectionError("No Serial Ports can be Found")
            SERIAL_PORT = ports[0]

        # Create OSC client & connect to Serial port
        print(f"Opening serial port {SERIAL_PORT} at {SERIAL_BAUD}")
        osc_client = udp_client.SimpleUDPClient(OSC_DESTINATION_IP, OSC_DESTINATION_PORT)        
        hardware = serial.Serial(port=SERIAL_PORT, baudrate=SERIAL_BAUD, timeout=.1)

        # Create OSC server process
        server_addr = (OSC_SERVER_IP, OSC_SERVER_PORT)
        parent_conn, child_conn = multiprocessing.Pipe()
        msg_queue = multiprocessing.Queue()

        osc_proc = multiprocessing.Process(target=proc_osc_server, args=(child_conn,msg_queue,server_addr,))
        osc_proc.start()
        

        try:
            run_main_loop(osc_client=osc_client, serial_conn=hardware, osc_msg_queue=msg_queue)
        except:
            # Stop main loop & osc_proc
            osc_proc.join()


