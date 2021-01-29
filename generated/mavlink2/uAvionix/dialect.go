//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package uavionix

// UavionixAdsbOutDynamicState (generated enum)
// State flags for ADS-B transponder dynamic report
const (
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE        = 1  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED    = 2  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND            = 8  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT                = 16 //
)

// UavionixAdsbOutRfSelect (generated enum)
// Transceiver RF control flags for ADS-B transponder dynamic reports
const (
	UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY    = 0 //
	UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1 //
	UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2 //
)

// UavionixAdsbOutDynamicGpsFix (generated enum)
// Status for ADS-B transponder dynamic input
const (
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D     = 2 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D     = 3 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS   = 4 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK    = 5 //
)

// UavionixAdsbRfHealth (generated enum)
// Status flags for ADS-B transponder dynamic output
const (
	UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0  //
	UAVIONIX_ADSB_RF_HEALTH_OK           = 1  //
	UAVIONIX_ADSB_RF_HEALTH_FAIL_TX      = 2  //
	UAVIONIX_ADSB_RF_HEALTH_FAIL_RX      = 16 //
)

// UavionixAdsbOutCfgAircraftSize (generated enum)
// Definitions for aircraft size
const (
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA     = 0  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M   = 1  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M     = 3  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M     = 4  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M     = 5  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M   = 6  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M     = 7  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M     = 8  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M     = 9  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M   = 10 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M     = 11 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M  = 12 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M    = 13 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M    = 14 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M    = 15 //
)

// UavionixAdsbOutCfgGpsOffsetLat (generated enum)
// GPS lataral offset encoding
const (
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA  = 0 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M  = 1 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M  = 2 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M  = 3 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7 //
)

// UavionixAdsbOutCfgGpsOffsetLon (generated enum)
// GPS longitudinal offset encoding
const (
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA           = 0 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1 //
)

// UavionixAdsbEmergencyStatus (generated enum)
// Emergency status encoding
const (
	UAVIONIX_ADSB_OUT_NO_EMERGENCY                    = 0 //
	UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY               = 1 //
	UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY             = 2 //
	UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY          = 3 //
	UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY               = 4 //
	UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5 //
	UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY       = 6 //
	UAVIONIX_ADSB_OUT_RESERVED                        = 7 //
)