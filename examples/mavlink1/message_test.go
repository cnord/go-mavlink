package mavlink

import (
	"bytes"
	"github.com/stretchr/testify/require"
	"math/rand"
	"testing"
)

func TestRoundTripChannels(t *testing.T) {
	rand.Seed(43)
	var buffer bytes.Buffer
	dec := NewDecoder(&buffer)

	pings := make([]CommonPing, 0, 255)
	processed := make([]CommonPing, 0, 255)

	for i := 0; i < 255; i++ {
		ping := CommonPing{
			Seq: rand.Uint32(),
		}
		pings = append(pings, ping)
		packet := &Packet{}
		require.Nil(t, packet.Encode(&ping), "Encode failed")
		buffer.Write(packet.Bytes())
	}
	var packet Packet
	for {
		if err := dec.Decode(&packet); err != nil {
			break
		}
		require.Equal(t, packet.MsgID, MSG_ID_PING, "MsgID fail")
		var ping CommonPing
		require.NoError(t, ping.Unpack(&packet), "Unpack fail")
		processed = append(processed, ping)
		if len(processed) == 255 {
			break
		}
	}
	require.Equal(t, len(pings), len(processed), "Pings not processed")
	for i, v := range pings {
		require.Equal(t, processed[i].Seq, v.Seq, "Order failed")
	}
}

func TestDecode(t *testing.T) {
	// decode a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E}))
	require.NoError(t, dec.Decode(&Packet{}))
}

func TestDecodeTwoMessages(t *testing.T) {
	// decode a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40}))
	var a, b Packet
	require.NoError(t, dec.Decode(&a))
	require.NoError(t, dec.Decode(&b))
	require.NotEqual(t, a, b)
}

func TestDecodeFalseStart(t *testing.T) {
	// a false start followed by a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E}))
	require.NoError(t, dec.Decode(&Packet{}))
}

func TestDecodeMultipleFalseStarts(t *testing.T) {
	// two false starts followed by a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40}))
	var a, b Packet
	require.NoError(t, dec.Decode(&a))
	require.NoError(t, dec.Decode(&b))
	require.NotEqual(t, a, b)
}
