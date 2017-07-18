#! /usr/bin/env python
import socket
import struct
import atexit
import signal
import sys
import math
import cmath
import time
from flask import Flask, render_template, request
from datetime import datetime
import socket
import json
from collections import OrderedDict
import subprocess
import os

if socket.gethostname().find('.')>=0:
    hostname=socket.gethostname()
else:
    hostname=socket.gethostbyaddr(socket.gethostname())[0]
hostname = hostname.split('.')[0]

app = Flask(__name__)

usb_devices = []
processes = []
sockets = []
connected = False

client_ip = "127.0.0.1"
client_port = 4242


#TODO: User Guide and Functions, Read Commands should contain 'X A'. Where X and A are internal frequency, auxiliary frequency, time constant etc., Write commands should contain 'X=Y A=B'. Where X and A are internal frequency, auxiliary frequency, time constant etc. and Y and B are their values

#################### FLASK STUFF - USER TO CLIENT #############################

@app.route('/')
def index():
    refresh_usb_devices()
    resource_table = format_devices_table(usb_devices)
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    return render_template('index.html', resource_table = resource_table, hostname = hostname, current_time = current_time)

@app.route("/connect")
def connect_device():
    global connected
    refresh_usb_devices()
    data = 'NOT CONNECTED'
    errors = ''
    device_port = ''
    x=0
    while not connected and x<3:
        try:
            x+=1
            if connected:
                raise Exception('You are already connected')
            # script = 'python /home/pi/lockin/lockin.py {} {} {}'.format(usb_devices[0], client_ip, str(client_port))
            script = 'python lockin.py {} {} {}'.format(usb_devices[0], client_ip, str(client_port))
            processes.append(subprocess.Popen([script], bufsize=1024, shell=True))
            time.sleep(9)
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((client_ip, client_port))
            sockets.append(client_socket)
            device_port = client_socket.recv(1024)
            time.sleep(0.1)
            client_socket.send("B")
            data = 'CONNECTED TO: {}'.format(device_port)
            connected = True
        except Exception as e:
            errors = str(e)
    output = OrderedDict([('command', 'connect'), ('data', data), ('errors', errors)])
    return json.dumps(output)

@app.route("/disconnect")
def disconnect_device():
    global connected, processes
    data = 'CONNECTED'
    errors = ''
    try:
        if not connected:
            raise Exception('You are not connected.')
        client_socket = sockets[0]
        client_socket.send("X")
        time.sleep(5)
        client_socket.shutdown(socket.SHUT_RDWR)
        while 1:
            try:
                byte = client_socket.recv(1)
                if "" == byte:
                    break
            except:
                break
        for subprocess in processes:
            if subprocess.poll() is None:
                subprocess.kill()
        del processes[:]
        client_socket.close()
        del sockets[:]
        connected = False
        data = "DISCONNECTED"
    except Exception as f:
        errors = str(f)
    output = OrderedDict([('command', 'disconnect'), ('data', data), ('errors', errors)])
    return json.dumps(output)

@app.route("/read", methods=['POST', 'GET'])
def read():
    global connected
    data = ''
    errors = ''
    command = ''
    try:
        if connected:
            if request.method == 'POST':
                post = request.form
                if post['command']:
                    command=post['command']
                else:
                    raise ValueError('You did not send a read command')
            else:
                raise ValueError('Please POST a command')
            data = device_read(command)
        else:
            raise ValueError('Please first connect to a device')
    except Exception as e:
        errors = str(e)
    output = OrderedDict([('command', command), ('data', data), ('errors', errors)])
    return json.dumps(output)

@app.route("/write", methods=['POST', 'GET'])
def write():
    data = ''
    errors = ''
    command = ''
    try:
        if request.method == 'POST':
            post = request.form
            if post['command']:
                command=post['command']
            else:
                raise ValueError('You did not send a command')
        else:
            raise ValueError('Please POST an address and command')
        data = device_write(command)
    except Exception as e:
        errors = str(e)
    output = OrderedDict([('command', command), ('data', data), ('errors', errors)])
    return json.dumps(output)

################################### FLASK HELPER METHODS & CLIENT TO LOCKIN ####################################

def refresh_usb_devices():
    global usb_devices
    import sys
    import glob
    import serial
    # Which USB device to connect to
    usb_devices = []
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        return EnvironmentError('Unsupported platform')
    for port in ports:
        try:
            if "usb" in port or "USB" in port:
                s = serial.Serial(port)
                s.close()
                usb_devices.append(port)
        except (OSError, serial.SerialException):
            return serial.SerialException

def format_devices_table(devlist):
    """ put devices list into HTML table """
    table = '<table border="1" cellpadding="5" cellspacing="5">'
    table += '<caption>List of available resources:</caption>'
    table += '<tr> <th>Device(s)</th> </tr>'
    for i in devlist:
        table += '<tr> <td>{}</td> </tr>'.format(i)
    table += '</table>'
    return table

def device_read(cmd):
    data = {}
    client_socket = sockets[0]
    client_socket.send("R")
    time.sleep(0.01)
    iq = client_socket.recv(1024)
    if len(iq) == 0:
        print "Could not get data from device"
    try:
        i,q = struct.unpack("dd", iq[1:17])
    except Exception as ex:
        print "something bad happened"
    f = struct.unpack("d", iq[17:25])[0]
    h = struct.unpack("d", iq[25:33])[0]
    a = struct.unpack("i", iq[33:37])[0]
    tc = struct.unpack("i", iq[37:41])[0]
    num = struct.unpack("i", iq[41:45])[0]
    g1 = struct.unpack("i", iq[45:49])[0]
    g2 = struct.unpack("i", iq[49:53])[0]
    g3 = struct.unpack("i", iq[53:57])[0]
    phase = struct.unpack("d", iq[57:65])[0]
    flags = struct.unpack("i", iq[65:69])[0]
    if flags & 128:
        ref = "External Reference"
    else:
        ref = "Internal Clock Reference"
    if flags & 3 == 1 or flags & 3 == 2:
        status = "Unlocked!"
    elif flags & 8:
        status = "Input overload"
    elif connected == True:
        status = "Running"
    else:
        status = "Stopped"

    if "RI" in cmd:
        data.update({"Real":str(i), "Imaginary":str(q)})
    if "MP" in cmd:
        val_1 = (i**2+q**2)**0.5
        if (val_1 == 0.):
            dec_1 = 1
        else:
            dec_1 = 3-int(math.log10(val_1))
        if dec_1 < 0:
            dec_1 = 0
        #print float(q), float(i)
        if float(q) == 0. and float(i) == 0:
            val_2 = 0.
        else:
            val_2 = 180/math.pi*math.atan2(float(q),float(i))
        #print val_2
        if (val_2 == 0.):
            dec_2 = 1
        else:
            dec_2 = 3-int(math.log10(abs(val_2)))
        #print dec_2
        if dec_2 < 0:
            dec_2 = 0
        data.update({"Magnitude":(("%."+str(dec_1)+"f") % val_1), "Phase":(("%."+str(dec_2)+"f") % val_2)})
    if 'F' in cmd:
        data.update({"Frequency":f})
    if 'AF' in cmd:
        data.update({"Aixiliary Frequency":h})
    if 'A' in cmd:
        data.update({"Amplitude":a})
    if 'TC' in cmd:
        data.update({"Time Constant":tc})
    if 'NS' in cmd:
        data.update({"Number of Samples":num})
    if 'G1' in cmd:
        data.update({"Gain 1":g1})
    if 'G2' in cmd:
        data.update({"Gain 2":g2})
    if 'G3' in cmd:
        data.update({"Gain 3":g3})
    if 'PS' in cmd:
        data.update({"Phase":phase})
    if 'R' in cmd:
        data.update({"Reference":ref})
    if 'S' in cmd:
        data.update({"Status":status})
    return data

def device_write(commands):
    cmds=commands.split(" ")
    client_socket = sockets[0]
    output = ''
    g1 = ""
    g2 = ""
    g3 = ""
    for cmd in cmds:
        status = ''
        param, val = cmd.split("=")
        if param == 'F':
            try:
                client_socket.send("F{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'AF':
            try:
                client_socket.send("G{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'A':
            try:
                client_socket.send("C{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'TC': ##Needs to be in miliseconds
            try:
                client_socket.send("T{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'NS':
            try:
                client_socket.send("S{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'PS':
            try:
                client_socket.send("P{}".format(val))
                status = 'success'
            except Exception as e:
                status = e
        elif param == 'R':
            if val == "I" or val == "J":
                try:
                    client_socket.send(val)
                    status = 'success'
                except Exception as e:
                    status = e
        elif param == 'G1':
            g1 = val;
        elif param == 'G2':
            g2 = val
        elif param == 'G3':
            g3 = val
        else:
            raise ValueError('Please POST a correct command')
        output += "{}: {}\n".format(cmd, status)
    if g1 and g2 and g3:
        try:
            client_socket.send("H {} {} {}".format(g1,g2,g3));
            status = 'Gains: success'
        except Exception as e:
            status = e
        output += "{}\n".format(status)
    elif g1 or g2 or g3:
        output += "Please proved all three gains (G1, G2, G3)\n"
    return output

def exit_signal_handler(signal, frame):
    if connected:
        disconnect_device()
    sys.exit(1)
    time.sleep(1)

############################# Run script ######################################

if __name__ == "__main__":
    signal.signal(signal.SIGINT, exit_signal_handler)
    app.run(threaded=True, debug=True, host='0.0.0.0')
