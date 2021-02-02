package main

import (
	"flag"
	"github.com/asmyasnikov/go-mavlink/common"
	_ "github.com/asmyasnikov/go-mavlink/generated/mavlink1/ardupilotmega"
	_ "github.com/asmyasnikov/go-mavlink/generated/mavlink2/ardupilotmega"
	"log"
	"net"
	"sync"
)

//////////////////////////////////////
//
// mavlink udp rx example
//
// listen to ardupilot SITL and prints received msgs, more info:
// http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/
//
// run via `go run main.go -mavlink 1`
//
//////////////////////////////////////

var (
	address = flag.String("address", ":14550", "address to listen on")
)

func main() {
	flag.Parse()
	listenAndServe(*address)
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

	decs := common.Decoders(conn)
	wg := sync.WaitGroup{}
	wg.Add(len(decs))
	for i := range decs {
		dec := decs[i]
		go func() {
			defer wg.Done()
			log.Println("listening packets from decoder " + dec.Name())
			for {
				if p, err := dec.Decode(); err != nil {
					log.Fatal(p)
				} else {
					log.Println(p)
				}
			}
		}()
	}
}
