package main

import (
	"../mavlinkSwitcher"
	"flag"
	"log"
	"net"
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
	mavlink = flag.Int("mavlink", 2, "version of mavlink (1 or 2)")
	rxaddr = flag.String("addr", ":14550", "address to listen on")
)

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

	dec := mavlinkSwitcher.Init(conn, *mavlink)
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
