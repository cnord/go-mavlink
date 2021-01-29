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
	"bufio"
	"encoding/hex"
	"flag"
	"io"
	"log"
	"os"
	"regexp"
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
	hexInput = flag.Bool("h", false, "process input stream as hex dump")
)

type hexByteReader struct {
	r *bufio.Reader
}

func (r *hexByteReader) Read(p []byte) (n int, err error) {
	b, err := r.r.ReadBytes('\n')
	if err != nil {
		return 0, err
	}
	reg, err := regexp.Compile("[^a-fA-F0-9]+")
	if err != nil {
		return 0, err
	}
	b, err = hex.DecodeString(reg.ReplaceAllString(string(b), ""))
	copy(p, b)
	return len(p), err
}

func main() {
	flag.Parse()
	listenAndServe()
}

func listenAndServe() {
	info, err := os.Stdin.Stat()
	if err != nil {
		panic(err)
	}
	if info.Mode()&os.ModeNamedPipe == 0 {
		log.Println("The command is intended to work with pipes.")
		return
	}
	var reader io.Reader
	if *hexInput {
		reader = &hexByteReader{
			r: bufio.NewReader(os.Stdin),
		}
	} else {
		reader = bufio.NewReader(os.Stdin)
	}
	dec := common.NewDecoder(reader)
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
