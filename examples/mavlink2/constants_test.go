package mavlink

import (
	"github.com/stretchr/testify/require"
	"testing"
)

func TestU16ToBytes(t *testing.T) {
	expected := []byte{0x25, 0x93}
	bytes := u16ToBytes(37669)
	require.Equal(t, len(bytes), 2, "Len of bytes should be equal 2")
	require.Equal(t, expected[0], bytes[0], "First byte unexpected")
	require.Equal(t, expected[1], bytes[1], "Second byte unexpected")
}
