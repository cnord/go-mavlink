package main

import (
	"../mavlinkSwitcher"
	"bufio"
	"flag"
	"log"
	"os"
)

//////////////////////////////////////
//
// mavlink pipe parser example
//
// listen to input pipe and prints received msgs, more info:
//
// run via `cat /dev/ttyUSB0 | go run main.go -version 1`
//
//////////////////////////////////////

var (
	mavlink = flag.Int("version", 2, "version of mavlink (1 or 2)")
)

func main() {
	flag.Parse()
	listenAndServe(*mavlink)
}

func listenAndServe(version int) {
	info, err := os.Stdin.Stat()
	if err != nil {
		panic(err)
	}
	if info.Mode()&os.ModeNamedPipe == 0 {
		log.Println("The command is intended to work with pipes.")
		return
	}
	reader := bufio.NewReader(os.Stdin)
	dec := mavlinkSwitcher.Init(reader, version)
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
