package mavlink

import (
	"github.com/stretchr/testify/require"
	"math/rand"
	"sort"
	"sync"
	"testing"
	"time"
)

func TestRoundTripChannels(t *testing.T) {

	AddDialect(DialectArdupilotmega)

	rand.Seed(43)

	cases := []struct{ seq uint32 }{}
	for i := 0; i < 100; i++ {
		cases = append(cases, struct{ seq uint32 }{rand.Uint32()})
	}

	dec := NewChannelDecoder()
	defer dec.Stop()

	wg := &sync.WaitGroup{}
	wg.Add(2)

	go func(cases []struct{ seq uint32 }) {
		defer wg.Done()
		for i, _ := range cases {
			seq := cases[i].seq
			p := CommonPing{
				Seq: seq,
			}
			packet := &Packet{}
			require.Nil(t, p.Pack(packet), "Pack fail")
			packet.fixChecksum(dialects)
			dec.PushData(packet.Bytes())
		}
	}(cases)

	sort.Slice(cases, func(i, j int) bool {
		return cases[i].seq < cases[j].seq
	})

	go func() {
		defer wg.Done()
		for packet := dec.NextPacket(time.Millisecond); packet != nil && len(cases) > 0; packet = dec.NextPacket(time.Millisecond) {
			require.Equal(t, packet.MsgID, MSG_ID_PING, "MsgID fail")
			var pingOut CommonPing
			require.Nil(t, pingOut.Unpack(packet), "Unpack fail")
			i := sort.Search(len(cases), func(i int) bool {
				return cases[i].seq >= pingOut.Seq
			})
			require.NotEqual(t, i, len(cases), "Search case by seq = %d fail. Cases: %+v", pingOut.Seq, cases)
			require.Equal(t, cases[i].seq, pingOut.Seq, "Seq fail")
			cases = append(cases[:i], cases[i+1:]...)
		}
		require.Equal(t, 0, len(cases), "Cases not cleared")
	}()

	wg.Wait()
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

func TestDialects(t *testing.T) {
	dec := NewChannelDecoder()
	defer dec.Stop()

	// try to encode an ardupilot msg before we've added that dialect,
	// ensure it fails as expected
	mi := &ArdupilotmegaMeminfo{
		Brkval:  1000,
		Freemem: 10,
	}

	RemoveDialect(DialectArdupilotmega)

	packet := &Packet{}
	require.Equal(t, packet.Encode(0x1, 0x1, mi), ErrUnknownMsgID, "encode expected ErrUnknownMsgID")

	// add the dialect, and ensure it succeeds
	AddDialect(DialectArdupilotmega)
	require.Nil(t, packet.Encode(0x1, 0x1, mi), "encode unexpected err")

	dec.PushData(packet.Bytes())
	packet = dec.NextPacket(time.Millisecond)
	require.NotNil(t, packet, "Decode fail")
	var miOut ArdupilotmegaMeminfo
	require.Nil(t, miOut.Unpack(packet), "Unpack fail")
	require.Equal(t, miOut.Brkval, mi.Brkval, "Round trip fail")
	require.Equal(t, miOut.Freemem, mi.Freemem, "Round trip fail")
}
