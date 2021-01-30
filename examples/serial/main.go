package main

import (
	_ "../../common"
	mavlink "../../generated/mavlink1"
	"../../generated/mavlink1/ardupilotmega"
	"../../generated/mavlink1/common"
	"../../generated/mavlink1/minimal"
	"flag"
	"github.com/tarm/serial"
	"io"
	"log"
	"sync"
	"time"
)

//////////////////////////////////////
//
// mavlink serial port device reader and parser example
//
// listen serial port device and prints received msgs, more info:
//
// run via `go run main.go -d /dev/ttyACM0 -b 57600 -1`
//
//////////////////////////////////////

const (
	RETRY_COUNT = 2
)

var (
	baudrate = flag.Int("b", 57600, "baudrate of serial port connection")
	device   = flag.String("d", "/dev/ttyUSB0", "path of serial port device")
	ro = flag.Bool("ro", false, "read-only mode")
)

func main() {
	flag.Parse()
	port, err := serial.OpenPort(&serial.Config{
		Name:        *device,
		Baud:        *baudrate,
		ReadTimeout: time.Second,
		Size:        8,
		Parity:      serial.ParityNone,
		StopBits:    1,
	})
	if err != nil {
		log.Fatalf("Error on opening device %s: %s\n", device, err)
	}
	wg := sync.WaitGroup{}
	wg.Add(1)
	go listenAndServe(&wg, port)
	if !*ro {
		wg.Add(1)
		go handshake(&wg, port)
	}
	wg.Wait()
}

func listenAndServe(wg *sync.WaitGroup, device io.ReadWriteCloser) {
	defer wg.Done()
	dec := mavlink.NewDecoder(device)
	if dec == nil {
		log.Fatal("Nil decoder")
		return
	}
	log.Println("listening packets from decoder")
	var packet mavlink.Packet
	for {
		if err := dec.Decode(&packet); err != nil {
			log.Fatal(err)
		} else {
			log.Println("<-", packet.String())
		}
		if packet.MsgID == common.MSG_ID_TIMESYNC {
			ts := common.Timesync{}
			if err := ts.Unpack(&packet); err != nil {
				log.Fatal(err)
			} else {
				sendPacket(device, makeTimeSync(ts.Tc1))
			}
		}
	}
}

func makeHeartbeat() *mavlink.Packet {
	return makePacket(&minimal.Heartbeat{
		CustomMode:     0,
		Type:           minimal.MAV_TYPE_GCS,
		Autopilot:      minimal.MAV_AUTOPILOT_INVALID,
		BaseMode:       0,
		SystemStatus:   0,
		MavlinkVersion: 3,
	})
}

func makeRequestDataStream(msgID uint8, rate uint16) *mavlink.Packet {
	return makePacket(&common.RequestDataStream{
		ReqMessageRate:  rate,
		TargetSystem:    1,
		TargetComponent: 1,
		ReqStreamID:     uint8(msgID),
		StartStop:       1,
	})
}

func makeTextArray(text string) (bytes [50]byte) {
	copy(bytes[:], text)
	return bytes
}

func makePayload(payload []byte) (bytes [251]byte) {
	copy(bytes[:], payload)
	return bytes
}

func makeStatustext(text string) *mavlink.Packet {
	return makePacket(&common.Statustext{
		Severity: common.MAV_SEVERITY_INFO,
		Text:     makeTextArray(text),
	})
}

func makeCommandLong(cmd uint16, param1 uint32) *mavlink.Packet {
	return makePacket(&common.CommandLong{
		Param1:          1,
		Param2:          0,
		Param3:          0,
		Param4:          0,
		Param5:          0,
		Param6:          0,
		Param7:          0,
		Command:         cmd,
		TargetSystem:    1,
		TargetComponent: 1,
		Confirmation:    0,
	})
}

func makeParamRequestList() *mavlink.Packet {
	return makePacket(&common.ParamRequestList{
		TargetSystem:    1,
		TargetComponent: 1,
	})
}

func makeTimeSync(ts int64) *mavlink.Packet {
	return makePacket(&common.Timesync{
		Tc1: time.Now().UnixNano()*1000000,
		Ts1: ts,
	})
}

func makeFileTransferProtocol(payload []byte) *mavlink.Packet {
	return makePacket(&common.FileTransferProtocol{
		TargetNetwork:   0,
		TargetSystem:    1,
		TargetComponent: 1,
		Payload:         makePayload(payload),
	})
}

var seq = uint8(0)

func nextSeq() uint8 {
	seq++
	return seq
}

func makePacket(message mavlink.Message) *mavlink.Packet {
	packet := mavlink.Packet{
		SeqID:         nextSeq(),
		SysID:         255,
		CompID:        minimal.MAV_COMP_ID_MISSIONPLANNER,
	}
	if err := message.Pack(&packet); err != nil {
		log.Fatalf("Error on pack message: %s\n", err)
	}
	return &packet
}

func sendPacket(writer io.Writer, packet *mavlink.Packet) {
	for i := 0; i < RETRY_COUNT; i++ {
		bytes := packet.Bytes()
		if n, err := writer.Write(bytes); err != nil {
			log.Fatalf("Error on write packet: %s\n", err)
		} else if n != len(bytes) {
			log.Fatalf("Writed %d bytes but need write %d bytes\n", n, len(bytes))
		} else {
			log.Println("->", packet.String())
		}
		time.Sleep(time.Millisecond * 500)
	}
}

// https://mavlink.io/en/guide/mavlink_version.html#version_handshaking
func handshake(wg *sync.WaitGroup, writer io.Writer) {
	defer wg.Done()
	time.Sleep(time.Second)
	sendPacket(writer, makeCommandLong(common.MAV_CMD_REQUEST_PROTOCOL_VERSION, 1))
	sendPacket(writer, makeCommandLong(common.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTENDED_STATUS, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_POSITION, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA1, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA2, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA3, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_RAW_SENSORS, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_RC_CHANNELS, 2))
	sendPacket(writer, makeHeartbeat())
	sendPacket(writer, makeStatustext("Mission Planner 1.3.74"))
	sendPacket(writer, makeCommandLong(ardupilotmega.MAV_CMD_DO_SEND_BANNER, 0))
	sendPacket(writer, makeFileTransferProtocol([]byte{0, 0, 0, 4, 16, 0, 0, 0, 0, 0, 0, 0, 64, 80, 65, 82, 65, 77, 47, 112, 97, 114, 97, 109, 46, 112, 99, 107, 0}))
}
