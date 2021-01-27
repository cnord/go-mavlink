package mavlink

import (
	"github.com/stretchr/testify/require"
	"math/rand"
	"sync"
	"testing"
	"time"
)

func TestRoundTripChannels(t *testing.T) {

	AddDialect(DialectArdupilotmega)

	rand.Seed(43)

	dec := NewChannelDecoder()
	defer dec.Stop()

	wg := &sync.WaitGroup{}
	wg.Add(2)

	pings := make([]ArdupilotmegaPing, 0, 255)
	processed := make([]ArdupilotmegaPing, 0, 255)

	go func() {
		defer wg.Done()
		for i := 0; i < 255; i++ {
			ping := ArdupilotmegaPing{
				Seq: rand.Uint32(),
			}
			mtx.Lock()
			pings = append(pings, ping)
			mtx.Unlock()
			packet := &Packet{}
			require.Nil(t, packet.EncodeMessage(&ping), "Encode failed")
			dec.PushData(packet.Bytes())
			time.Sleep(time.Millisecond)
		}
	}()

	go func() {
		defer wg.Done()
		end := time.Now().Add(time.Second)
		for {
			packet := dec.NextPacket(time.Until(end))
			if packet == nil {
				break
			}
			require.Equal(t, packet.MsgID, MSG_ID_PING, "MsgID fail")
			var ping ArdupilotmegaPing
			require.Nil(t, ping.Unpack(packet), "Unpack fail")
			processed = append(processed, ping)
			if len(processed) == 255 {
				break
			}
		}
	}()
	wg.Wait()
	require.Equal(t, len(pings), len(processed), "Pings not processed")
	for i, v := range pings {
		require.Equal(t, processed[i].Seq, v.Seq, "Order failed")
	}
}

func TestDecode(t *testing.T) {
	// decode a known good byte stream
	dec := NewChannelDecoder()
	defer dec.Stop()
	dec.PushData([]byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E})
	require.NotNil(t, dec.NextPacket(time.Millisecond), "Decode fail")
}

func TestDecodeTwoMessages(t *testing.T) {
	// decode a known good byte stream
	dec := NewChannelDecoder()
	defer dec.Stop()
	dec.PushData([]byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40})
	msg1 := dec.NextPacket(time.Millisecond)
	require.NotNil(t, msg1, "Decode fail")
	msg2 := dec.NextPacket(time.Millisecond)
	require.NotNil(t, msg2, "Decode fail")
	require.NotEqual(t, *msg1, *msg2, "Messages should not match")
}

func TestDecodeFalseStart(t *testing.T) {
	// a false start followed by a known good byte stream
	dec := NewChannelDecoder()
	defer dec.Stop()
	dec.PushData([]byte{0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E})
	require.NotNil(t, dec.NextPacket(time.Millisecond), "Decode fail")
}

func TestDecodeMultipleFalseStarts(t *testing.T) {
	// two false starts followed by a known good byte stream
	dec := NewChannelDecoder()
	defer dec.Stop()
	dec.PushData([]byte{0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40})
	msg1 := dec.NextPacket(time.Millisecond)
	require.NotNil(t, msg1, "Decode fail")
	msg2 := dec.NextPacket(time.Millisecond)
	require.NotNil(t, msg2, "Decode fail")
	require.NotEqual(t, *msg1, *msg2, "Messages should not match")
}
