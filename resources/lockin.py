#!/usr/bin/python

import serial
import atexit
import math
import socket
import select
import struct
import sys
import signal
import re
from threading import Thread,Semaphore
from time import sleep

DIGIPOTS=False

receiving_data = False
running = True
socket_threads = []
sockets = []
num_stages = 4
i_rc = [0]*4
q_rc = [0]*4
k = 192e-6/.05 # 50 ms
mk = 1.-k
actual_frequency = 0.
actual_auxfrequency = 0.
amplitude = 0
gain1 = 0
gain2 = 0
gain3 = 0
all_gain = 1
time_constant = 50
phase = 0
mysem = Semaphore(1)
flags = 0 # one byte.

unostatus = 0 # the status byte from the uno32
overload_count = 0
unlock_count = 0
unlock_state = 0

cap_prescale = 0. #reported values using input capture
cap_period = 20.

# first bits same as uno status: bit 0 and 1 are input capture status
# - 0 means using internal oscillator, 01 and 10 are looking for lock
# 11 is locked.
# bit 2 is missed interrupt (unused?)
# bit 3 is input overload
# bit 7 is whether we think we're on internal oscillator or external ref.
# bit 7 in unostatus signals 'extended' information: new freq, new phase

#TODO: Cleanup an comment
#TODO: User adjust amplitude

def exit_handler():
    port.write("J")
    sleep(0.01)
    port.write("E")
    sleep(0.01)
    port.flush()
    port_thread.join()
    port.close()
    for thread in socket_threads:
        thread.join()
    for s in sockets:
        s.shutdown(socket.SHUT_RDWR)
        while 1:
            try:
                byte = s.recv(1)
                if "" == byte:
                    break
            except:
                break
        s.close()
    print("exiting lockin script")
    sys.exit(1)

def signal_handler(signal, frame):
    global running
    running = False

def port_listen_loop():
    global cap_prescale,cap_period,actual_frequency,actual_auxfrequency,unostatus,flags,overload_count,unlock_count,unlock_state, running
    while running:
        print('Checkpoint 1')
        length = 0
        while (length == 0):
            header = port.read(1)
            length = len(header)
            print('Checkpoint 2')
            print(length)
            if not running:
                return
        unostatus = ord(header)
        # take high bit of flags, rest of unostatus
        flags = (unostatus & 127) | (flags & 128)
        #deal with exception conditions - unlocked or overload,
        # these persist for 5000 input cycles.
        if unostatus & 8: # input overload
            overload_count = 5000
        if (unostatus & 3) == 1 or (unostatus & 3) == 2:
            unlock_count = 5000
            unlock_state = unostatus & 3
        if overload_count:
            overload_count -= 1
            flags |= 8
        if unlock_count:
            unlock_count -= 1
            flags &= ~3
            flags |= unlock_state
        if unostatus >15 and unostatus < 128:
           print "got header:",unostatus
        if ((unostatus & 128)  == 0 ):
            # high bit not set. This is a status, sample data follows
            i_str = port.read(2)
            while len(i_str) != 2:
                i_str = port.read(2)
            # print "reading i_str"
            i_int = struct.unpack("h", i_str)[0]
            q_str = port.read(2)
            while len(q_str) != 2:
                q_str = port.read(2)
            #print "reading q_str"
            q_int = struct.unpack("h", q_str)[0]
            mysem.acquire()
            i_rc[0] = (k*i_int + mk*i_rc[0])
            q_rc[0] = (k*q_int + mk*q_rc[0])
            for fi in range(1, num_stages):
                i_rc[fi] = (k*i_rc[fi-1] + mk*i_rc[fi])
                q_rc[fi] = (k*q_rc[fi-1] + mk*q_rc[fi])
            mysem.release()
            print('Checkpoint 3')
        else: # read the next byte to see what the message will be
            print('Checkpoint 4')
            head2 = ord(port.read(1))
            # 2: new phase
            #    read 2 bytes
            if head2 == 2:
                print 'header 2'
                p = port.read(2)
                print "new_phase:", ord(p[0])+256*ord(p[1])
                #print "p", [ord(c) for c in p]
            # 3: new frequency
            #    read 4 bytes
            elif head2 == 3:
                cap = port.read(2)
                cap_period = ord(cap[0])+ord(cap[1])*256
                #print "cap_period", [ord(c) for c in cap_period]
                print "cap_period", cap_period
                if (cap_period != 0):
                    actual_frequency = (80000000./2.**cap_prescale)/cap_period
                    print "set actual_frequency to:",actual_frequency
                    sys.stdout.flush()
            # 4-7: unused? should ask Carl about this
            elif head2 == 4:
                deb1 = ord(port.read(1))
                deb1 += ord(port.read(1))*256
                print 'deb1:',deb1
            elif head2 == 5:
                deb2 = ord(port.read(1))
                deb2 += ord(port.read(1))*256
                print 'deb2:',deb2
            elif head2 == 6:
                ldeb1 = ord(port.read(1))
                ldeb1 += ord(port.read(1))*256
                ldeb1 += ord(port.read(1))*256*256
                ldeb1 += ord(port.read(1))*256*256*256
                if ldeb1 > 1<<31:
                    ldeb1 -= (1<<32)
                print ldeb1
            # 8: new prescale
            #    read 2 bytes
            elif head2 == 8:
                cap = port.read(2)
                cap_prescale = ord(cap[0])
                if cap_period != 0:
                    actual_frequency = (80000000./2.**cap_prescale)/cap_period
                    print "cap_prescale", cap_prescale
                    print "set actual_frequency to:",actual_frequency
                    sys.stdout.flush()
            elif head2 == 9: #version info
                ver1 = port.read(1)
                ver2 = port.read(1)
                print "Firmware reports version:",str(ord(ver1))+"."+str(ord(ver2))
            elif head2 == 10: #averaged frequency
                ntoaverage = port.read(4)
                ntoaverage = struct.unpack("I",ntoaverage)[0]
                total_counts = port.read(4)
                total_counts = struct.unpack("I",total_counts)[0]
                actual_frequency = (80000000./2**cap_prescale)/total_counts*ntoaverage
                print "got averaged frequency.",ntoaverage,"averages, ",total_counts," counts.", actual_frequency," Hz"
            elif head2 == 42: # TODO just here for debugging
                size = port.read(1)
                print "size", ord(size)
                for i in range(ord(size)):
                    print ord(port.read(1))
            else:
                dummy = 42
                print "unknown header", header
            print('Checkpoint 5')
    print port_name + ": exiting listen loop"

def set_freq(command,frequency):
    #command is a letter or two, value is a float
    if frequency == 0:
        return 0 # this should never happen
    period = int(80000000/frequency)
    print "period", period
    prescale = 0
    if (period > 65535):
        prescale = int(math.log(period/65536)/math.log(2)+1)
    #print "raw prescale", prescale
        if prescale < 0:
            prescale = 0
        if prescale == 7: # hardware won't do 7.
            prescale = 8
        period /= 2**prescale
        #round period so it is divisible by 2 (XXX should this be 4?)
        #    period = int((period/2)*2)
        #na, don't bother.
        period =int(period+0.5)
    print "prescale", prescale
    print "period", period
    print "closest freq:", 80000000.0/(2**prescale)/period
    actual = 80000000.0/(2**prescale)/period
    print "actual was: ",actual
    if prescale > 8:
        print "prescale is > 8, bailing"
        return
    #command = freq_to_string(data,prescale,period)
    #print "sending",command
    for byte in command+chr(prescale)+chr(period & 255)+chr(period >> 8):
        port.write(byte)
        print "writing", ord(byte)
        sleep(0.02)
    return actual

def socket_listen_loop(s):
    global num_stages, i_rc, q_rc, receiving_data,actual_frequency, actual_auxfrequency, amplitude, gain1, gain2, gain3, time_constant,phase,all_gain,flags,prescale,cap_freq, running
    while running:
        data = s.recv(1024)
        # list of data headers
        # B: begin data collection
        # E: end data collection
        # F: set frequency
        # G: set aux frequency
        # X: exit
        # N: number of stages in low-pass filter
        # C: amplitude of sine output
        # H: gain of input amplifiers
        # R: server request for data
        # Q: quick request for data
        # I: internal reference
        # J: external reference
        if data:
        #print "received", data, "from", s.getpeername()
            # B: begin data collection
            if data[0] == "B":
                port.write("B")
                receiving_data = True
            # E: end data collection
            elif data[0] == "E":
                port.write("E")
                receiving_data = False
            # F: set frequency
            elif data[0] == "F":
                if flags & 128: #was on external osc.
                    flags &= ~128
                    port.write("J") #turn off ext osc.
                actual_frequency = set_freq("EF",float(data[1:]))
                if receiving_data == True:
                    print "writing B"
                    port.write("B")
            # G: set aux frequency
            elif data[0] == "G":
                actual_auxfrequency = set_freq("G",float(data[1:]))
	        # X: exit
            elif data[0] == "X":
                running = False
                break
            # N: number of stages in low-pass filter
            elif data[0] == "S":
                mysem.acquire()
                num_stages = int(data[1:])
                i_rc = [0]*num_stages
                q_rc = [0]*num_stages
                mysem.release()
    	    #T: time constant of low-pass filter
    	    elif data[0] == "T":
                time_constant = int(data[1:])
              	print "Time constant is",time_constant ,"ms"
                global k,mk
                k = 192e-6/(time_constant/1e3) # comes in ms.
                mk = 1.-k
    	    #P: phase
    	    elif data[0] == "P":
                phase = float(data[1:])*math.pi/180
              	print "Phase is",phase

            # C: amplitude
            elif data[0] == "C":
                # turn float into two bytes - coarse and fine.
                # this could almost certainly be improved.
                amplitude = float(data[1:])
                # amplitude = 5 * (val / 1024) in volts
                # use as DAC A as coarse, DAC B as fine
                #amp = 5V (maybe?) *(DAC A)* (DAC B)/255/255
                valA = int(amplitude/5000.*255 +0.999)
                # only use top 6 bits? always turn on bottom 2.
                valA = valA | 3
                if valA > 255:
                    valA = 255
                if valA > 0:
                    valB = int(amplitude/5000./valA*255*255)
                else:
                    valB = 0
                if valB > 255:
                    valB = 255
                print "using valA, valB",valA,valB,"Expect amp of:",5.*valA*valB/255/255,"V"
                for byte in "C"+chr(valA)+chr(valB):
                    port.write(byte)
                    print "writing", ord(byte)
                    sleep(0.02)

            # H: gain
            elif data[0] == "H":
                tokens = data.split()
                gain1 = int(math.log10(int(tokens[1])))
                gain2 = int(math.log(int(tokens[2]))/math.log(2))
                gain3 = int(math.log10(int(tokens[3])))
                all_gain = int(tokens[1])*int(tokens[2])*int(tokens[3])
                print "all_gain is:",all_gain
                gain_cat = (gain1 & 0x3) | ((gain2 & 0x3) << 2) | ((gain3 & 0x3) << 4)
		print gain1,"\n"
		print gain2,"\n"
		print gain3,"\n"
		print gain_cat
                for byte in "A" + chr(gain_cat):
                    port.write(byte)
                    print "writing", ord(byte)
                    sleep(0.02)
                print "done writing gain"
            # R: server request for data
            # TODO: add bytes for second filter, amplitude, gain stages, time constant, filter order
            # data is composed of following:
            #     byte 1: "Y" if data is being received from the uno, "N" otherwise
            #     bytes 2-9: real/imaginary parts of filter
            #     bytes 10-17: frequency
            elif data[0] == "R":
#this is sent to the gui in the flags byte:
#                if unostatus & 3 == 1 or unostatus &3 == 2:
#                    print 'unlocked'
#                if unostatus & 8:
#                    print "Input Overload"
                m = ""
                if receiving_data:
                    m += "Y"
                    iout = (i_rc[num_stages-1] *math.cos(phase)+q_rc[num_stages-1]*math.sin(phase))*4./all_gain # 4 here keeps scaling same as before
                    qout = (i_rc[num_stages-1] *-1*math.sin(phase)+q_rc[num_stages-1]*math.cos(phase))*4./all_gain
                    mflags = flags
                else:
                    m += "N"
                    iout = 0.
                    qout = 0.
                    #now, the input overload flag is only valid if the adc
                    #is running, so if not, mask it out
                    mflags = flags & 247 # 247 = 255-8.
                mysem.acquire()

                m += struct.pack("d", iout)+ struct.pack("d", qout)
                mysem.release()
                m += struct.pack("d", actual_frequency)
                m += struct.pack("d", actual_auxfrequency)
		m += struct.pack("i", amplitude)
		m += struct.pack("i", time_constant)
		m += struct.pack("i", num_stages)
		m += struct.pack("i", gain1) + struct.pack("i", gain2) + struct.pack("i", gain3)
		m += struct.pack("d", phase*180/math.pi)
                m += struct.pack("i",mflags)

#               	print m
                if running:
#                    print "sending", [ord(i) for i in m]
                    s.send(m)
                else:
                    print "not sending", [ord(i) for i in m]
            # Q: quick request for data
            # only send 8 bytes of R/I filter values
            # TODO: this should be 16 bytes, from two filters
            elif data[0] == "Q":
                if receiving_data:
                    iout = (i_rc[num_stages-1] *math.cos(phase)+q_rc[num_stages-1]*math.sin(phase))*4./all_gain # 4 here keeps scaling same as before
                    qout = (i_rc[num_stages-1] *-1*math.sin(phase)+q_rc[num_stages-1]*math.cos(phase))*4./all_gain
                else:
                    iout = 0.
                    qout = 0.
                m = struct.pack("d", iout)+struct.pack("d", qout)
                if running:
                    print "sending", [ord(i) for i in m], "(Q)"
                    s.send(m)
                else:
                    print "not sending", [ord(i) for i in m], "(Q)"
            # N: internal clock trigger for adc
            elif data[0] == "I":
                print "sending I"
                flags |= 128;
                port.write("I")
            # L: external reference trigger for adc
            elif data[0] == "J":
                print "sending J"
                flags &= ~128;
                port.write("J")
        #socket.send(data)
    print "exiting socket listen loop"

port_name = sys.argv[1]
client_ip = sys.argv[2]
client_port = int(sys.argv[3])

for i in range(3):
    try:
        port = serial.Serial(port_name, 2000000, timeout = 1.5)
    except Exception as ex:
        print "Connection to", port_name, "failed"
        raise ex#sys.exit(1)
        print port_name + ": looking for lock-in"
    sleep(5)
    port.write('Q')
    port.flush()
    response = port.read(11)
    print "to Q query, received:",response
    if response[0:9] == "LCK READY":
        i=20
        break
    port.write('E')
    port.write('J')
    port.flush()
    sleep(0.1)
    port.flush()
    port.close()
    sleep(5) # it seems to need this long if you open it at just the wrong time.
if i != 20:
    print "Serial port failure"
    sys.exit(1)

if DIGIPOTS:
    for j in range(3):
        try:
            port = serial.Serial(port_name, 2000000, timeout = 0.5)
        except Exception as ex:
            print "Connection to", port_name, "failed"
            raise ex#sys.exit(1)
            print port_name + ": looking for digipots"
        port.write('D')
        port.flush()
        response = port.read(11)
        print "to D query, received:",response
        if response[0:9] == "LCK READY":
            j=20
            break
        port.write('E')
        port.write('J')
        port.flush()
        sleep(0.1)
        port.flush()
        port.close()
        sleep(20) # it seems to need this long if you open it at just the wrong time.
    if j != 20:
        print "digipots failure"
        sys.exit(1)

# Start up in known state
port.write("E") # not running.
sleep(.01)

port.write("A") # set gain settings to 0
sleep(.01)
port.write(chr(0))
sleep(.01)
actual_frequency = set_freq("F",50.0)
print "set actual_frequency to",actual_frequency
actual_auxfrequency = set_freq("G",5000.)
print "set actual_auxfrequency to",actual_auxfrequency

#do this after the F, since J will cause the lock-in to report its frequency
port.write("J") # no input capture on start-up
sleep(.01)

port.write("V") # request version number

#this is for start-up value of output. Want output amplitude set to minimum
# this is for DACS, not digipots.
#amplitude = 0
#val = 0
#for byte in "C"+chr(val & 255)+chr(val >> 8):
#    port.write(byte)
#    print "writing", ord(byte)
#    sleep(0.01)
#for byte in "D"+chr(val & 255)+chr(val >> 8):
#    port.write(byte)
#    print "writing", ord(byte)
#    sleep(0.01)


port_thread = Thread(target=port_listen_loop)
port_thread.daemon = True
port_thread.start()

atexit.register(exit_handler)
signal.signal(signal.SIGINT, signal_handler)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# we shouldn't care about whether an address is being reused
# note that if packets were being sent when the server was shut down, they may stay in a phantom state for a minute or two
# thus some weird data may show up if the program(s) previously failed to exit nicely
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((client_ip, client_port))
server_socket.listen(5)

if running:
    try:
        conn, addr = server_socket.accept()
        print "connecting to", conn, addr
        # send name
        conn.send(port_name)
        socket_thread = Thread(target=socket_listen_loop, kwargs={"s": conn})
        socket_threads.append(socket_thread)
        sockets.append(conn)
        socket_thread.daemon = True
        socket_thread.start()
    except Exception as e:
        print(e)
