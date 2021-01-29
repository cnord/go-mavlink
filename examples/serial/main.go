package main

import (
	"../../common"
	_ "../../generated/mavlink1/ardupilotmega"
	_ "../../generated/mavlink1/common"
	_ "../../generated/mavlink1/icarous"
	_ "../../generated/mavlink1/minimal"
	_ "../../generated/mavlink1/uAvionix"
	_ "../../generated/mavlink2/ardupilotmega"
	_ "../../generated/mavlink2/common"
	_ "../../generated/mavlink2/icarous"
	_ "../../generated/mavlink2/minimal"
	_ "../../generated/mavlink2/uAvionix"
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
	baudrate = flag.Int("b", 57600, "baudrate of serial port connection")
	device   = flag.String("d", "/dev/ttyUSB0", "path of serial port device")
)

func main() {
	flag.Parse()
	listenAndServe(*device, *baudrate)
}

func listenAndServe(device string, baudrate int) {
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
	dec := common.NewDecoder(port)
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
