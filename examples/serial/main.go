package main

import (
	mavlink1 "../mavlink1"
	mavlink2 "../mavlink2"
	"bufio"
	"flag"
	"github.com/tarm/serial"
	"log"
	"sync"
	"time"
)

//////////////////////////////////////
//
// mavlink serial port device reader and parser example
//
// listen serial port device and prints received msgs, more info:
//
// run via `go run main.go -d /dev/ttyACM0 -b 57600 -v 1`
//
//////////////////////////////////////

var version = flag.Int("v", 2, "version of mavlink - 1 or 2")
var baudrate = flag.Int("b", 57600, "baudrate of serial port connection")
var device = flag.String("d", "/dev/ttyUSB0", "path of serial port device")

func main() {
	flag.Parse()
	listenAndServe(*version, *device, *baudrate)
}

type Decoder interface {
	PushData(data []byte)
	Stop()
}

func mavlink1Decoder() (Decoder, chan interface{}) {
	mavlink1.AddDialect(mavlink1.DialectArdupilotmega)
	dec := mavlink1.NewChannelDecoder()
	ch := make(chan interface{})
	go func() {
		for p := range dec.DecodedChannel() {
			msg, ok := mavlink1.DialectArdupilotmega.GetMessage(p)
			if ok {
				ch <- msg
			} else {
				log.Printf("Bad mavlink1 packet %+v", p)
			}
		}
		close(ch)
	}()
	log.Println("select mavlink1 decoder")
	return dec, ch
}

func mavlink2Decoder() (Decoder, chan interface{}) {
	mavlink2.AddDialect(mavlink2.DialectArdupilotmega)
	dec := mavlink2.NewChannelDecoder()
	ch := make(chan interface{})
	go func() {
		for p := range dec.DecodedChannel() {
			msg, ok := mavlink2.DialectArdupilotmega.GetMessage(p)
			if ok {
				ch <- msg
			} else {
				log.Printf("Bad mavlink2 packet %+v", p)
			}
		}
		close(ch)
	}()
	log.Println("select mavlink2 decoder")
	return dec, ch
}

func initDecoder(version int) (Decoder, chan interface{}) {
	switch version {
	case 1:
		return mavlink1Decoder()
	case 2:
		return mavlink2Decoder()
	default:
		log.Printf("undefined version (%d) of mavlink decoder\n", version)
		return nil, nil
	}
}

func listenAndServe(version int, device string, baudrate int) {
	dec, ch := initDecoder(version)
	if dec == nil || ch == nil {
		return
	}
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
	wg := &sync.WaitGroup{}
	wg.Add(2)
	go func() {
		defer wg.Done()
		log.Printf("listening serial port device %s", device)
		scanner := bufio.NewScanner(port)
		scanner.Split(bufio.ScanBytes)
		for scanner.Scan() {
			data := scanner.Bytes()
			dec.PushData(data)
		}
		if err := scanner.Err(); err != nil {
			log.Fatal(err)
		}
		log.Println("EOF")
		dec.Stop()
	}()
	go func() {
		defer wg.Done()
		log.Println("listening packets from decoder")
		for p := range ch {
			log.Printf("Receive package %+v\n", p)
		}
	}()
	wg.Wait()
}
