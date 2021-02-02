package common

import (
	"github.com/stretchr/testify/require"
	"testing"
)

func TestMavlinkVersion(t *testing.T) {
	require.Equal(t, MAVLINK_V1, mavlinkVersion(true, false))
	require.Equal(t, MAVLINK_V2, mavlinkVersion(false, true))
	require.Equal(t, MAVLINK_V1|MAVLINK_V2, mavlinkVersion(true, true))
	require.Equal(t, MAVLINK_V1|MAVLINK_V2, mavlinkVersion(false, false))
}
