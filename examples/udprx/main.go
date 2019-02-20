package main

import (
	"flag"
	"log"
	"net"
	"sync"
	"time"

	"../mavlink1"
)

//////////////////////////////////////
//
// mavlink udp rx example
//
// listen to ardupilot SITL and prints received msgs, more info:
// http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/
//
// run via `go run main.go`
//
//////////////////////////////////////

var rxaddr = flag.String("addr", ":14550", "address to listen on")

func main() {

	flag.Parse()

	listenAndServe(*rxaddr)
}

func listenAndServe(addr string) {

	udpAddr, err := net.ResolveUDPAddr("udp", addr)
	if err != nil {
		log.Fatal(err)
	}

	conn, listenerr := net.ListenUDP("udp", udpAddr)
	if listenerr != nil {
		log.Fatal(listenerr)
	}

	log.Println("listening on", udpAddr)

	dec := mavlink.NewChannelDecoder()

	wg := &sync.WaitGroup{}
	wg.Add(2)
	go func() {
		defer wg.Done()
		for {
			buffer := make([]byte, 0, 255)
			n, err := conn.Read(buffer)
			if err != nil {
				log.Print(err)
			} else if n > 0 {
				dec.PushData(buffer[:n])
			}
		}
	}()
	go func() {
		defer wg.Done()
		for {
			packet := dec.NextPacket(time.Second)
			if packet != nil {
				log.Println(*packet)
			}
		}
	}()

	mavlink.AddDialect(mavlink.DialectArdupilotmega)

	wg.Wait()
}
