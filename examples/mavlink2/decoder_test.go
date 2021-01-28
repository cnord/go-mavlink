package mavlink

import (
	"bytes"
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
		for i := 0; i < b.N; i++ {
			dummy := ping{
				Seq: rand.Uint32(),
			}
			packet := &Packet{}
			if err := packet.encode(uint8(rand.Uint32()%uint32(^uint8(0))), uint8(rand.Uint32()%uint32(^uint8(0))), &dummy); err != nil {
				b.Fatal(err)
			}
			buffer.Write(packet.Bytes())
			sended++
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
