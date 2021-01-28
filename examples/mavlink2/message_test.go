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

	pings := make([]pingMock, 0, 255)
	processed := make([]pingMock, 0, 255)

	for i := 0; i < 255; i++ {
		ping := pingMock{
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
		require.Equal(t, packet.MsgID, MSG_ID_PING_MOCK, "MsgID fail")
		var ping pingMock
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
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xf, 0x27, 0x7, 0xc5}))
	require.NoError(t, dec.Decode(&Packet{}))
}

func TestDecodeTwoMessages(t *testing.T) {
	// decode a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xf, 0x27, 0x7, 0xc5,
		0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xe8, 0x3, 0xf0, 0x4}))
	var a, b Packet
	require.NoError(t, dec.Decode(&a))
	require.NoError(t, dec.Decode(&b))
	require.NotEqual(t, a, b)
}

func TestDecodeFalseStart(t *testing.T) {
	// a false start followed by a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xFD, 0x0D, 0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xf, 0x27, 0x7, 0xc5}))
	require.NoError(t, dec.Decode(&Packet{}))
}

func TestDecodeMultipleFalseStarts(t *testing.T) {
	// two false starts followed by a known good byte stream
	dec := NewDecoder(bytes.NewBuffer([]byte{0xFD, 0x0D, 0xFD, 0x0D, 0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xf, 0x27, 0x7, 0xc5,
		0x00, 0xFD, 0x2B, 0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xe8, 0x3, 0xf0, 0x4}))
	var a, b Packet
	require.NoError(t, dec.Decode(&a))
	require.NoError(t, dec.Decode(&b))
	require.NotEqual(t, a, b)
}
