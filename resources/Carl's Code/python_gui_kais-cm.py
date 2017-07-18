#!/usr/bin/python2

import socket
import struct
import atexit
import sys
import math
import cmath
import time
import pygtk
pygtk.require("2.0")
internalGuiUpdate = 0

gui_running = True
request_running = False
status_label = None
# ping server every 0.25 seconds
def server_request_loop():
    global internalGuiUpdate,gui_running
    #print "starting sr loop"
    if gui_running:
        client_socket.send("R")
        time.sleep(0.01)
        iq = client_socket.recv(1024)
        if len(iq) == 0:
            print "server quit?"
            gui_running = False # stops the read thread.
            status_label.set_text("Server connection lost")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(red=1.0))
            return True
        if iq[0] == "Y":
            if not request_running:
                internalGuiUpdate = 1
                builder.get_object("operation_button").set_active(True)
                internalGuiUpdate = 0
        else:
            if request_running:
                internalGuiUpdate = 1
                builder.get_object("operation_button").set_active(False)
                internalGuiUpdate = 0

                # set data
        try:
            i,q = struct.unpack("dd", iq[1:17])
            #                    print "received", i, q
            if builder.get_object("radio_button_ri").get_active():
                builder.get_object("display_coarse_1").set_text(str(i))
                builder.get_object("display_coarse_2").set_text(str(q))
            else:
                val_1 = (i**2+q**2)**0.5
                if (val_1 == 0.):
                    dec_1 = 1
                else:
                    dec_1 = 3-int(math.log10(val_1))
                if dec_1 < 0:
                    dec_1 = 0
                #print dec_1, dec_1 < 0, type(dec_1), int(math.log10(val_1)), val_1, ("%."+str(dec_1)+"f") % val_1
                builder.get_object("display_coarse_1").set_text(("%."+str(dec_1)+"f") % val_1)
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
                builder.get_object("display_coarse_2").set_text(("%."+str(dec_2)+"f") % val_2)
        except Exception as ex:
            print "something bad happened"
            status_label.set_text("Error!")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(red=1.0))
            # set frequency
		
        #set the gui to reflect everything else in the data packet.
        internalGuiUpdate = 1
        f = struct.unpack("d", iq[17:25])[0]
        print "frequency", f
        builder.get_object("actual_frequency_num").set_label(str(f))
        h = struct.unpack("d", iq[25:33])[0]
        print "auxfrequency", h
        builder.get_object("actual_auxfrequency_num").set_label(str(h))
        a = struct.unpack("i", iq[33:37])[0]
        builder.get_object("adjustment_amplitude").set_value(a)
        print "amplitude", a
        
        time_constant = struct.unpack("i", iq[37:41])[0]
        print "time constant", time_constant
        if time_constant > 500: 
            time_constant/= 1000
            builder.get_object("time_constant_unit_box").set_active(1)
        else:
            builder.get_object("time_constant_unit_box").set_active(0) 
        active = int(math.log10(time_constant)*3+0.5) #convert to menu index!
        builder.get_object("time_constant_num_box").set_active(active) 


        num = struct.unpack("i", iq[41:45])[0]
        print "number of stages", num
        builder.get_object("num_stages_box").set_active(num/2-1)

        g1 = struct.unpack("i", iq[45:49])[0]
        print "gain1", g1
        g2 = struct.unpack("i", iq[49:53])[0]
        print "gain2", g2
        g3 = struct.unpack("i", iq[53:57])[0]
        print "gain3", g3
        builder.get_object("gain_stage_1_box").set_active(g1)
        builder.get_object("gain_stage_2_box").set_active(g2)
        builder.get_object("gain_stage_3_box").set_active(g3)
        
        phase = struct.unpack("d", iq[57:65])[0]
        print "phase", phase
        builder.get_object("adjustment_phase").set_value(phase)
        flags = struct.unpack("i", iq[65:69])[0]
        print "got flags: ",flags
        if flags & 128:
            builder.get_object("radio_button_er").set_active(True)
        else:
            builder.get_object("radio_button_ic").set_active(True)
        internalGuiUpdate = 0
        if flags & 3 == 1 or flags & 3 == 2:
            status_label.set_text("Unlocked!")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(red=1.0))
        elif flags & 8:
            status_label.set_text("Input overload")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(red=1.0))
        elif request_running == True:
            status_label.set_text("Running")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(green=1.0))
        else:
            status_label.set_text("Stopped")
            status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color())
        #print [ord(c) for c in i], struct.unpack("i", i)
        #cs = select.select([client_socket], [], [], 1)
        #if cs[0]:
        #q = client_socket.recv(4)
        #print [ord(c) for c in q], struct.unpack("i", q)
        return True
    else: #gui_running is false, quit out.
        return False 
curr_width = -1
curr_height = -1

def check_resize(widget, *args):
    global curr_width, curr_height
    #print "resized", widget
    # resize data display
    font_width = builder.get_object("display_coarse_1").get_allocation().width
    if curr_width != font_width:
        curr_width = font_width
        for name in ["coarse","fine"]:
            obj = builder.get_object("display_"+name+"_1")
            obj.modify_font(pango.FontDescription("monospace "+str(int(font_width/5.5))))
            obj = builder.get_object("display_"+name+"_2")
            obj.modify_font(pango.FontDescription("monospace "+str(int(font_width/11))))
    """image = builder.get_object("image1")
    width = image.get_allocation().width
    height = image.get_allocation().height
    pixbuf = gtk.gdk.pixbuf_new_from_file("0.svg")
    scaled_buf = pixbuf.scale_simple(image.get_allocation().width, image.get_allocation().height,gtk.gdk.INTERP_BILINEAR)
    if curr_width != width and curr_height != height:
        curr_width = width
        curr_height = height
        image.set_from_pixbuf(scaled_buf)"""

def delete_event(widget, event, *args):
    print "quitting"
    gtk.main_quit()

def operation_changed(togglebutton, *args):
    global request_running
    if togglebutton.get_active():
        print "starting"
        if not internalGuiUpdate:
            client_socket.send("B")
        togglebutton.set_label("ON")
        request_running = True
#        status_label.set_text("Running")
#        status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color(green=1.0))
    else:
        print "ending"
        if not internalGuiUpdate:
            client_socket.send("E")
        togglebutton.set_label("OFF")
        request_running = False
#        status_label.set_text("Stopped")
#        status_label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.Color())

def gain_stage_changed(combobox, *args):
    p1 = builder.get_object("gain_stage_1_box").get_active_text()
    p2 = builder.get_object("gain_stage_2_box").get_active_text()
    p3 = builder.get_object("gain_stage_3_box").get_active_text()
    total_gain = reduce(lambda x,y: x*y, [int(i) for i in [p1, p2, p3]])
    print p1, p2, p3, total_gain
    builder.get_object("total_gain_num").set_text(str(total_gain))
    if not internalGuiUpdate:
        client_socket.send("H"+ " " + p1 + " " + p2 + " " +p3);

def num_stages_changed(combobox, *args):
    if not internalGuiUpdate:
        print "num stages now", int(builder.get_object("num_stages_box").get_active_text())
        client_socket.send("S"+builder.get_object("num_stages_box").get_active_text())
def time_constant_changed(combobox, *args):
    print "IN time_constant_changed"
    if not internalGuiUpdate:
        print "time_constant", int(builder.get_object("time_constant_num_box").get_active_text()),builder.get_object("time_constant_unit_box").get_active_text()
        tc = int(builder.get_object("time_constant_num_box").get_active_text())
        if builder.get_object("time_constant_unit_box").get_active_text() == "s":
            tc *= 1000 # always send in units of ms.
        print "SENDING tc of:",tc
        client_socket.send("T"+str(tc))

def trigger_changed(radiobutton, *args):
    if not internalGuiUpdate:
        if builder.get_object("radio_button_ic").get_active():
            print "internal clock"
            client_socket.send("J")
        else:
            print "external reference"
            client_socket.send("I")



def frequency_changed(adjustment, *args):
    # just dump the string to server
    client_socket.send("F"+str(adjustment.get_value()))
    while gtk.events_pending():
        gtk.main_iteration()
    time.sleep(.15)
def auxfrequency_changed(adjustment, *args):
    client_socket.send("G"+str(adjustment.get_value()))
    while gtk.events_pending():
        gtk.main_iteration()
    time.sleep(.15)
def amplitude_changed(adjustment, *args):
    # pass string to server
    if not internalGuiUpdate:
        client_socket.send("C"+str(adjustment.get_value()))
        time.sleep(.15)
def phase_changed(adjustment, *args):
    # pass string to server
    if not internalGuiUpdate:
        client_socket.send("P"+str(adjustment.get_value()))
        while gtk.events_pending():
            gtk.main_iteration()
        time.sleep(.15)


try:
    import gtk
    import gtk.glade
    import pango
except:
    print "GTK import failed - is PyGTK installed?"
    sys.exit(1)

port_name = ""
# TODO: to remove socket code to test gui:
 # comment here
def exit_handler():
    global gui_running
    print "closing socket and sending X"
    gui_running = False
#    server_request_thread.join()
    client_socket.send("X")
    time.sleep(0.1)
    client_socket.close()

if len(sys.argv) > 2:
    client_ip = sys.argv[1]
    client_port = int(sys.argv[2])
else:
    client_ip = "127.0.0.1"
    client_port = 4242
print "IP:", client_ip
print "Port:", client_port

# may need to change ip and such
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    client_socket.connect((client_ip, client_port))
    time.sleep(0.1)
    port_name = client_socket.recv(1024)
except socket.error:
    print "Connection failed - are you sure the server is running?"
    sys.exit(1)

atexit.register(exit_handler)

#read, write, err = select.select([client_socket], [client_socket], [], 60)
#if write:
#    write[0].connect(("localhost", 5000))
#print "connected"

 # uncomment here
builder = gtk.Builder()
builder.add_from_file("guikais.glade")
builder.connect_signals(locals())

# set port name
builder.get_object("operation_label").set_text(port_name)


#builder.get_object("eventbox1").add(gtk.image_new_from_file("0.svg"))

for name in ["coarse","fine"]:
    for num in ["1","2"]:
        builder.get_object("display_"+name+"_"+num).modify_font(pango.FontDescription("monospace 42"))

window = builder.get_object("main_window")
window.modify_bg(gtk.STATE_NORMAL, gtk.gdk.color_parse("white"))
window.show_all()
status_label = builder.get_object("status_label")


# threading stuff is required so GTK doesn't lock everything
server_request_timeout = gtk.timeout_add(250,server_request_loop)
gtk.main()
