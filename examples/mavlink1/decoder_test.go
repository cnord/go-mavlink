package mavlink

import (
	"bytes"
	"fmt"
	"math/rand"
	"sync"
	"testing"
	"time"
)

func BenchmarkDecoder(b *testing.B) {
	var buffer bytes.Buffer
	dec := NewDecoder(&buffer)

	rand.Seed(123)

	wg := &sync.WaitGroup{}
	wg.Add(2)

	rand.Seed(time.Now().UnixNano())

	sended := uint32(0)
	received := uint32(0)
	go func() {
		defer wg.Done()
		fmt.Println("to send", b.N)
		for i := 0; i < b.N; i++ {
			dummy := CommonPing{
				Seq: rand.Uint32(),
			}
			packet := &Packet{}
			if err := packet.encode(uint8(rand.Uint32()%uint32(^uint8(0))), uint8(rand.Uint32()%uint32(^uint8(0))), &dummy); err != nil {
				b.Fatal(err)
			}
			fmt.Println("sended pre")
			buffer.Write(packet.Bytes())
			sended++
			fmt.Println("sended")
		}
	}()

	go func() {
		defer wg.Done()
		var packet Packet
		for {
			err := dec.Decode(&packet)
			if err == nil {
				return
			}
			received++
		}
	}()
	wg.Wait()
	if received != sended {
		b.Fatalf("Sended (%d) and received (%d) packets not equal", sended, received)
	}
}
