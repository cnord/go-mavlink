package main

import (
	_ "../../common"
	mavlink "../../generated/mavlink2"
	"../../generated/mavlink2/ardupilotmega"
	"../../generated/mavlink2/common"
	"../../generated/mavlink2/minimal"
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
	wg.Add(2)
	go listenAndServe(&wg, port)
	go handshake(&wg, port)
	wg.Wait()
}

func listenAndServe(wg *sync.WaitGroup, device io.Reader) {
	defer wg.Done()
	dec := mavlink.NewDecoder(device)
	if dec == nil {
		return
	}
	log.Println("listening packets from decoder")
	var packet mavlink.Packet
	for {
		if err := dec.Decode(&packet); err != nil {
			log.Fatal(err)
		} else {
			log.Println(packet.Message())
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

func makeCommandLong(cmd uint16) *mavlink.Packet {
	return makePacket(&common.CommandLong{
		Param1:          0,
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
		SeqID:  nextSeq(),
		SysID:  0,
		CompID: 0,
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
		}
	}
}

// https://mavlink.io/en/guide/mavlink_version.html#version_handshaking
func handshake(wg *sync.WaitGroup, writer io.Writer) {
	defer wg.Done()
	time.Sleep(time.Second)
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTENDED_STATUS, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_POSITION, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA1, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA2, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_EXTRA3, 4))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_RAW_SENSORS, 2))
	sendPacket(writer, makeRequestDataStream(common.MAV_DATA_STREAM_RC_CHANNELS, 2))
	sendPacket(writer, makeHeartbeat())
	sendPacket(writer, makeStatustext("Custom GCS"))
	sendPacket(writer, makeCommandLong(ardupilotmega.MAV_CMD_DO_SEND_BANNER))
	sendPacket(writer, makeFileTransferProtocol([]byte{0, 0, 0, 4, 16, 0, 0, 0, 0, 0, 0, 0, 64, 80, 65, 82, 65, 77, 47, 112, 97, 114, 97, 109, 46, 112, 99, 107, 0}))
}
