package main

import (
	"bufio"
	"flag"
	"io"
	"log"
	"os"
	"sync"

	mavlink1 "../mavlink1"
	mavlink2 "../mavlink2"
)

//////////////////////////////////////
//
// mavlink pipe parser example
//
// listen to input pipe and prints received msgs, more info:
//
// run via `cat /dev/ttyUSB0 | go run main.go -v 1`
//
//////////////////////////////////////

var version = flag.Int("v", 2, "version of mavlink - 1 or 2")

func main() {
	flag.Parse()
	listenAndServe(*version)
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
			ch <- p
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
			ch <- p
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

func listenAndServe(version int) {
	info, err := os.Stdin.Stat()
	if err != nil {
		panic(err)
	}
	if info.Mode()&os.ModeNamedPipe == 0 {
		log.Println("The command is intended to work with pipes.")
		return
	}
	dec, ch := initDecoder(version)
	if dec == nil || ch == nil {
		return
	}
	reader := bufio.NewReader(os.Stdin)
	wg := &sync.WaitGroup{}
	wg.Add(2)
	go func() {
		defer wg.Done()
		log.Println("listening stdin")
		for {
			r, _, err := reader.ReadRune()
			if err == io.EOF {
				log.Println("EOF. Exit")
				break
			} else if err != nil {
				log.Fatalf("Error on reading from input: %s\n", err)
			} else {
				log.Printf("Receive rune %q from input\n", r)
				dec.PushData([]byte(string(r)))
			}
		}
		dec.Stop()
	}()
	go func() {
		defer wg.Done()
		log.Println("listening packets from decoder")
		for p := range ch {
			log.Println(p)
		}
	}()
	wg.Wait()
}
