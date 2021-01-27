package mavlink

import (
	"math/rand"
	"sync"
	"testing"
	"time"
)

func BenchmarkDecoder(b *testing.B) {
	dec := NewChannelDecoder()

	rand.Seed(123)

	wg := &sync.WaitGroup{}
	wg.Add(2)

	rand.Seed(time.Now().UnixNano())

	counterOut := uint32(0)
	counterIn := uint32(0)
	go func() {
		defer wg.Done()
		defer dec.Stop()
		for i := 0; i < b.N; i++ {
			dummy := ArdupilotmegaPing{
				Seq: rand.Uint32(),
			}
			packet := &Packet{}
			if err := packet.Encode(uint8(rand.Uint32()%uint32(^uint8(0))), uint8(rand.Uint32()%uint32(^uint8(0))), &dummy); err != nil {
				b.Fatal(err)
			}
			dec.PushData(packet.Bytes())
			counterOut++
		}
	}()

	go func() {
		defer wg.Done()
		for {
			packet := <-dec.decoded
			if packet == nil {
				return
			}
			counterIn++
		}
	}()
	wg.Wait()
	if counterIn != counterOut {
		b.Fatalf("Sended (%d) and received (%d) packets not equal", counterOut, counterIn)
	}
}
