package main

import(
	"fmt"
	"time"
	"math"
	"strings"
	"unicode/utf8"
	"encoding/binary"
	"github.com/abiosoft/semaphore"
	// "github.com/tarm/serial"
	serial "github.com/jacobsa/go-serial/serial"
	s "github.com/mikepb/go-serial"
	)

var(
	receiving_data bool
	running bool
	// TODO: Threads and sockets
	// socket_threads = []
	// sockets = []
	num_stages int
	i_rc [4]int
	q_rc [4]int
	k float64
	mk float64
	actual_frequency float64
	actual_auxfrequency float64
	amplitude float64
	gain1 int
	gain2 int
	gain3 int
	all_gain int
	time_constant float64
	phase float64
	mysem = semaphore.New(1)
	flags int
	unostatus int//# the status byte from the uno32
	overload_count int
	unlock_count int
	unlock_state int
	cap_prescale float32//#reported values using input capture
	cap_period float64
)

//Funciton reads available usb serial ports. This library only lists dev/cu not dev/tty ports, so it also converts the 'cu' to 'tty'. It only selects a port with usb in it.
func get_port_address()(error, string){
	var port_address []byte
	usb := false
	ports, err := s.ListPorts()
	if err == nil {
		for _, info := range ports {
			port_name := []byte(info.Name())
			for x := 0; x < len(port_name); x++{
				if (port_name[x] == 'u' || port_name[x] == 'U') && (port_name[x+1] == 's' || port_name[x+1] == 'S') && (port_name[x+2] == 'b' || port_name[x+2] == 'B'){
					usb = true
				}
				if x == len(port_name)-1 && usb==false{
					port_address=nil
					break
				}
				if port_name[x] == 'c' && port_name[x+1] == 'u' {
					port_address = append(port_address,'t')
					port_address = append(port_address,'t')
					port_address = append(port_address,'y')
					x++
				}else{
					port_address = append(port_address,port_name[x])
				}
			}
		}
		return err, string(port_address)
	}else{
		return err, "error"
	}
}

//Function communicates with Uno32 and sets requencies
func set_freq(command string, frequency float64)(float64, []byte, []byte, []byte, []byte){
	//command is a letter or two, value is a float
	if frequency == 0{
		return 0, nil, nil, nil, nil //this should never happen
	}
	period := int(80000000.0/frequency)
	fmt.Printf("Period: %d\n", period)
	var prescale int
	if period > 65535 {
		prescale = int(math.Log(float64(period)/65536.0)/math.Log(2.0)+1)
		//print "raw prescale", prescale
		if prescale < 0{
			prescale=0
		}
		if prescale == 7{//hardware won't do 7.
			prescale = 8
		}
		// round period so it is divisible by 2 (XXX should this be 4?)
    // period = int((period/2)*2)
    // na, don't bother.
		period = int((float64(period)/(math.Exp2(float64(prescale))))+0.5)
	}
	actual := 80000000.0/math.Exp2(float64(prescale))/float64(period)
	fmt.Printf("Prescale: %d\nPeriod: %d\nCloset Freq: %f\nActual was: %f\n", prescale, period, actual, actual)
	if prescale > 8{
		fmt.Printf("Prescale is > 8, bailing")
		return 0, nil, nil, nil, nil
	}
	// command = freq_to_string(data,prescale,period)
	// print "sending",command
	cmd := []byte(command)
	p1 := []byte(string(prescale))
	p2 := []byte(string(period & 255))
	p3 := []byte(string(period >> 8))
	return actual, cmd, p1, p2, p3
}

//This functions countinously loops and communicates with uno32. It intializes the serial port in the beginning.
func port_listen_loop(port_loop_channel chan string){
	//Initializing Serial and starting Uno in known state
	var buffer []byte
Port_Init:
	tries:=3
	err, port_address := get_port_address()
	if err != nil{
		fmt.Printf("Error getting port address: %v\n", err)
	}
	// c := &serial.Config{Name: port_address, Baud: 115200, ReadTimeout: time.Second * 1}
  // port, err := serial.OpenPort(c)
  // if err != nil {
  // 	fmt.Printf("%v",err)
	// }
  options := serial.OpenOptions{PortName: port_address, BaudRate: 2000000, DataBits: 8, StopBits: 1, MinimumReadSize: 1,}
  port, err := serial.Open(options)
  if err != nil {
	  fmt.Printf("Error opening serial port: %v\n", err)
		if tries>0{
			port.Close()
			tries--
			goto Port_Init
		}else{
			goto Close_Port
		}
  }else{
		fmt.Printf("Successfully connected to: %s\n", port_address)
	}
	time.Sleep(5000 * time.Millisecond)
	port.Write([]byte("Q"))
	buffer = make([]byte, 11)
  port.Read(buffer)
	fmt.Printf("%s", buffer)
	if !strings.Contains(string(buffer), "LCK READY"){
		port.Write([]byte("E"))
    port.Write([]byte("J"))
    time.Sleep(500* time.Millisecond)
		if tries>0{
			port.Close()
			tries--
			goto Port_Init
		}else{
			goto Close_Port
		}
	}else{
		var p1 []byte
		var p2 []byte
		var p3 []byte
		port.Write([]byte("E"))
		time.Sleep(10* time.Millisecond)
		port.Write([]byte("A"))
		time.Sleep(10* time.Millisecond)
		port.Write([]byte("\000"))
		time.Sleep(10* time.Millisecond)
		actual_frequency, buffer, p1, p2, p3 = set_freq("F", 50.0)
		port.Write(buffer)
		time.Sleep(20* time.Millisecond)
		port.Write(p1)
		time.Sleep(20* time.Millisecond)
		port.Write(p2)
		time.Sleep(20* time.Millisecond)
		port.Write(p3)
		time.Sleep(20* time.Millisecond)
		fmt.Printf("set actual_frequency to %f\n",actual_frequency)
		actual_auxfrequency, buffer, p1, p2, p3 = set_freq("G",5000.)
		port.Write(buffer)
		time.Sleep(20* time.Millisecond)
		port.Write(p1)
		time.Sleep(20* time.Millisecond)
		port.Write(p2)
		time.Sleep(20* time.Millisecond)
		port.Write(p3)
		time.Sleep(20* time.Millisecond)
		fmt.Printf("set aux_frequency to %f\n",actual_auxfrequency)
		port.Write([]byte("J"))
		time.Sleep(10* time.Millisecond)
		port.Write([]byte("V"))
	}

	//Port listen loop

	// port_loop_channel <- "Running"
	// running = false
	for running{
		length := 0
		header := make([]byte, 1)
		for length==0{
			port.Read(header)
			length = len(header)
			if !running{
				goto Close_Port
			}
		}
		rune, _ := utf8.DecodeRune(header)
		unostatus = int(rune)
		//take high bit of flags, rest of unostatus
		flags = (unostatus & 127) | (flags & 128)
		//deal with exception conditions - unlocked or overload,
    //these persist for 5000 input cycles.
		if (unostatus & 8)!=0{
			overload_count = 5000
		}
		if (unostatus & 3) == 1 || (unostatus & 3) == 2{
			overload_count = 5000
			unlock_state = unostatus & 3
		}
		if overload_count!=0{
			overload_count--
			flags |= 8
		}
		if unlock_count!=0{
			unlock_count --
			flags &= ^3
			flags |= unlock_state
		}
		if unostatus > 15 && unostatus < 128{
			fmt.Print("Got header: %d", unostatus)
		}
		if (unostatus & 128) == 0{
			i_str := make([]byte, 2)
			port.Read(i_str)
			for len(i_str)!=2{
				port.Read(i_str)
			}
			//int16 for 2 bytes
			i_int :=int16(binary.LittleEndian.Uint16(i_str))
			q_str := make([]byte, 2)
			port.Read(q_str)
			for len(q_str)!=2{
				port.Read(q_str)
			}
			q_int :=int16(binary.LittleEndian.Uint16(q_str))
			mysem.Acquire()
			i_rc[0] = int(k*float64(i_int) + mk*float64(i_rc[0]))
      q_rc[0] = int(k*float64(q_int) + mk*float64(q_rc[0]))
			for fi := 1; fi < num_stages; fi++{
				i_rc[fi] = int(k*float64(i_rc[fi-1]) + mk*float64(i_rc[fi]))
        q_rc[fi] = int(k*float64(q_rc[fi-1] )+ mk*float64(q_rc[fi]))
			}
			mysem.Release()
			//loop gets stuck somewhere
		}else{
		}

	}

Close_Port:
	port.Close()
	port_loop_channel <- "Exiting Port Loop"
}

func init(){
	receiving_data = false
	running = true
		// TODO: Threads and sockets
		// socket_threads = []
		// sockets = []
	num_stages = 4
	k = 192e-6 / .05 // 50 ms
	mk = 1. - k
	actual_frequency = 0.
	actual_auxfrequency = 0.
	amplitude = 0
	gain1 = 0
	gain2 = 0
	gain3 = 0
	all_gain = 1
	time_constant = 50
	phase = 0
		// TODO: semaphore
		// mysem = Semaphore(1)
	flags = 0
	unostatus = 0 //# the status byte from the uno32
	overload_count = 0
	unlock_count = 0
	unlock_state = 0
	cap_prescale = 0. //#reported values using input capture
	cap_period = 20.
		// runtime.GOMAXPROCS(2)
}

func main() {
	//Channel for port communication
	port_loop_channel := make(chan string)

	go port_listen_loop(port_loop_channel)

	for {
		if running{
			port_feedback := <- port_loop_channel
			fmt.Printf("%s\n", port_feedback)
			if port_feedback == "Exiting Port Loop"{
				break
			}
		}else{
			goto Terminate
		}
	}

Terminate:
	fmt.Printf("Terminating\n")
}
