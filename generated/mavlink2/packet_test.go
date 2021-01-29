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
	ping := &pingMock{
		Seq: 1000,
	}
	packet := &Packet{}
	require.NoError(t, packet.Encode(ping), "encode failed")
	bytes, err := Marshal(packet)
	require.NoError(t, err)
	require.Equal(t, []byte{0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xe8, 0x3, 0xf0, 0x4}, bytes)
	require.NoError(t, Unmarshal([]byte{0xfd, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0xde, 0x0, 0x0, 0xe8, 0x3, 0xf0, 0x4}, packet))
}

func TestEncodeDecode(t *testing.T) {
	ping := &pingMock{
		Seq: 1000,
	}
	packet := &Packet{}
	require.NoError(t, packet.Encode(ping), "encode failed")
	require.NoError(t, packet.Decode(ping), "decode failed")
}
