package mavlink

import (
	"github.com/stretchr/testify/require"
	"testing"
)

func TestU16ToBytes(t *testing.T) {
	expected := []byte{0x25, 0x93}
	bytes := (*Packet)(nil).u16ToBytes(37669)
	require.Equal(t, len(bytes), 2, "Len of bytes should be equal 2")
	require.Equal(t, expected[0], bytes[0], "First byte unexpected")
	require.Equal(t, expected[1], bytes[1], "Second byte unexpected")
}

func TestMarshallUnmarshal(t *testing.T) {
	mi := &ArdupilotmegaMeminfo{
		Brkval:  1000,
		Freemem: 10,
	}

	packet := &Packet{}
	require.NoError(t, packet.Encode(mi), "encode failed")
	bytes, err := Marshal(packet)
	require.NoError(t, err)
	require.Equal(t, []byte{0xfe, 0x4, 0x0, 0x0, 0x0, 0x98, 0xe8, 0x3, 0xa, 0x0, 0xad, 0xbb}, bytes)
	require.NoError(t, Unmarshal([]byte{0xfe, 0x4, 0x0, 0x0, 0x0, 0x98, 0xe8, 0x3, 0xa, 0x0, 0xad, 0xbb}, packet))
}

func TestEncodeDecode(t *testing.T) {
	mi := &ArdupilotmegaMeminfo{
		Brkval:  1000,
		Freemem: 10,
	}

	packet := &Packet{}
	require.NoError(t, packet.Encode(mi), "encode failed")
	require.NoError(t, packet.Decode(mi), "decode failed")
}