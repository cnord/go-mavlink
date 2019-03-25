package mavlink

import (
	"github.com/stretchr/testify/require"
	"testing"
	"time"
)

func TestAddRemove(t *testing.T) {

	ds := DialectSlice{DialectCommon}

	// verify initial state
	require.Equal(t, len(ds), 1, "bad len before add")
	require.Equal(t, ds.IndexOf(DialectCommon), 0, "couldn't find dialect")

	// verify addition
	ds.Add(DialectArdupilotmega)
	require.Equal(t, len(ds), 2, "bad len after add")
	require.Equal(t, ds.IndexOf(DialectArdupilotmega), 1, "couldn't find dialect")

	// verify removal
	ds.Remove(DialectCommon)
	require.Equal(t, len(ds), 1, "bad len after remove")
	require.NotEqual(t, ds.IndexOf(DialectCommon), 0, "wrong dialect")
	require.GreaterOrEqual(t, ds.IndexOf(DialectArdupilotmega), 0, "wrong dialect")
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

	packet := &Packet{}
	require.Nil(t, packet.Encode(0x1, 0x1, mi), "encode failed")

	// remove the dialect, and ensure it failed
	RemoveDialect(DialectArdupilotmega)
	dec.PushData(packet.Bytes())
	decoded := dec.NextPacket(time.Millisecond * time.Duration(10))
	require.Nil(t, decoded, "Decode fail")

	// add the dialect, and ensure it succeeds
	AddDialect(DialectArdupilotmega)
	dec.PushData(packet.Bytes())
	decoded = dec.NextPacket(time.Millisecond * time.Duration(10))
	require.NotNil(t, decoded, "Decode fail")
	var miOut ArdupilotmegaMeminfo
	require.Nil(t, miOut.Unpack(packet), "Unpack fail")
	require.Equal(t, miOut.Brkval, mi.Brkval, "Round trip fail")
	require.Equal(t, miOut.Freemem, mi.Freemem, "Round trip fail")
}
