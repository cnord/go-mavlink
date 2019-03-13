package mavlink

import (
	"github.com/stretchr/testify/require"
	"testing"
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
	require.GreaterOrEqual(t, ds.IndexOf(DialectCommon), 0, "wrong dialect")
	require.NotEqual(t, ds.IndexOf(DialectArdupilotmega), 0, "wrong dialect")
}
