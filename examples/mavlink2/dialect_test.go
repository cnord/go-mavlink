package mavlink

import (
	"github.com/stretchr/testify/require"
	"testing"
	"time"
)

func TestAddRemove(t *testing.T) {

	ds := DialectSlice{DialectCommon}

	// verify initial state
	if len(ds) != 1 {
		t.Error("bad len after remove")
	}
	if ds.IndexOf(DialectCommon) != 0 {
		t.Error("couldn't find dialect")
	}

	// verify addition
	ds.Add(DialectArdupilotmega)
	if len(ds) != 2 {
		t.Error("bad len after add")
	}
	if ds.IndexOf(DialectArdupilotmega) != 1 {
		t.Error("couldn't find dialect")
	}

	// verify removal
	ds.Remove(DialectCommon)
	if len(ds) != 1 {
		t.Error("bad len after remove")
	}
	if ds.IndexOf(DialectCommon) >= 0 {
		t.Error("wrong dialect")
	}
	if ds.IndexOf(DialectArdupilotmega) != 0 {
		t.Error("wrong dialect")
	}
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
	decoded := dec.NextPacket(time.Millisecond)
	require.Nil(t, decoded, "Decode fail")

	// add the dialect, and ensure it succeeds
	AddDialect(DialectArdupilotmega)
	dec.PushData(packet.Bytes())
	decoded = dec.NextPacket(time.Millisecond)
	require.NotNil(t, decoded, "Decode fail")
	var miOut ArdupilotmegaMeminfo
	require.Nil(t, miOut.Unpack(packet), "Unpack fail")
	require.Equal(t, miOut.Brkval, mi.Brkval, "Round trip fail")
	require.Equal(t, miOut.Freemem, mi.Freemem, "Round trip fail")
}
