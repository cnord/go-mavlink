package main

import (
	"../mavlinkSwitcher"
	"flag"
	"github.com/tarm/serial"
	"log"
	"time"
)

//////////////////////////////////////
//
// mavlink serial port device reader and parser example
//
// listen serial port device and prints received msgs, more info:
//
// run via `go run main.go -d /dev/ttyACM0 -b 57600 -1`
//
//////////////////////////////////////

var (
	mavlink1 = flag.Bool("1", false, "mavlink 1")
	baudrate = flag.Int("b", 57600, "baudrate of serial port connection")
	device = flag.String("d", "/dev/ttyUSB0", "path of serial port device")
)

func main() {
	flag.Parse()
	mavlink := 2
	if mavlink1 != nil && *mavlink1 == true {
		mavlink = 1
	}
	listenAndServe(mavlink, *device, *baudrate)
}

func listenAndServe(mavlink int, device string, baudrate int) {
	port, err := serial.OpenPort(&serial.Config{
		Name:        device,
		Baud:        baudrate,
		ReadTimeout: time.Second,
		Size:        8,
		Parity:      serial.ParityNone,
		StopBits:    1,
	})
	if err != nil {
		log.Fatalf("Error on opening device %s: %s\n", device, err)
	}
	dec := mavlinkSwitcher.Init(port, mavlink)
	if dec == nil {
		return
	}
	log.Println("listening packets from decoder")
	for {
		if p, err := dec.Decode(); err != nil {
			log.Fatal(p)
		} else {
			log.Println(p)
		}
	}
}
