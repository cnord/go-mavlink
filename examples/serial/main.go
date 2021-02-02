package main

import (
	"flag"
	decoders "github.com/asmyasnikov/go-mavlink/common"
	mavlink "github.com/asmyasnikov/go-mavlink/generated/mavlink1"
	"github.com/asmyasnikov/go-mavlink/generated/mavlink1/ardupilotmega"
	_ "github.com/asmyasnikov/go-mavlink/generated/mavlink2/ardupilotmega"
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

var (
	baudrate = flag.Int("b", 57600, "baudrate of serial port connection")
	device   = flag.String("d", "/dev/ttyUSB0", "path of serial port device")
	ro       = flag.Bool("ro", false, "read-only mode")
	retryCount = flag.Int("retry-count", 2, "retry count for sending packets")
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
		log.Fatalf("Error on opening device %s: %s\n", *device, err.Error())
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
	decs := decoders.Decoders(device)
	for i := range decs {
		wg.Add(1)
		go func(dec decoders.Decoder) {
			defer wg.Done()
			log.Println("listening packets from decoder " + dec.Name())
			for {
				packet, err := dec.Decode()
				if err != nil {
					log.Fatal(dec.Name(), " error: ", err.Error())
				} else {
					log.Println("<-", packet.String())
				}
				//if packet.MsgID == ardupilotmega.MSG_ID_TIMESYNC {
				//	ts := ardupilotmega.Timesync{}
				//	if err := ts.Unpack(&packet); err != nil {
				//		log.Fatal(err)
				//	} else {
				//		sendPacket(device, makeTimeSync(ts.Ts1))
				//	}
				//}
			}
		}(decs[i])
	}
}

func makeHeartbeat() *mavlink.Packet {
	return makePacket(&ardupilotmega.Heartbeat{
		CustomMode:     0,
		Type:           ardupilotmega.MAV_TYPE_GCS,
		Autopilot:      ardupilotmega.MAV_AUTOPILOT_INVALID,
		BaseMode:       0,
		SystemStatus:   0,
		MavlinkVersion: 3,
	})
}

func makeRequestDataStream(msgID ardupilotmega.MAV_DATA_STREAM, rate uint16) *mavlink.Packet {
	return makePacket(&ardupilotmega.RequestDataStream{
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
	return makePacket(&ardupilotmega.Statustext{
		Severity: ardupilotmega.MAV_SEVERITY_INFO,
		Text:     makeTextArray(text),
	})
}

func makeCommandLong(cmd ardupilotmega.MAV_CMD, param1 uint32) *mavlink.Packet {
	return makePacket(&ardupilotmega.CommandLong{
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
	return makePacket(&ardupilotmega.ParamRequestList{
		TargetSystem:    1,
		TargetComponent: 1,
	})
}

func makeTimeSync(ts int64) *mavlink.Packet {
	return makePacket(&ardupilotmega.Timesync{
		Tc1: time.Now().UnixNano(),
		Ts1: ts,
	})
}

func makeFileTransferProtocol(payload []byte) *mavlink.Packet {
	return makePacket(&ardupilotmega.FileTransferProtocol{
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
		SysID:  255,
		CompID: uint8(ardupilotmega.MAV_COMP_ID_MISSIONPLANNER),
	}
	if err := message.Pack(&packet); err != nil {
		log.Fatalf("Error on pack message: %s\n", err)
	}
	return &packet
}

func sendPacket(writer io.Writer, packet *mavlink.Packet) {
	for i := 0; i < *retryCount; i++ {
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
	sendPacket(writer, makeCommandLong(ardupilotmega.MAV_CMD_REQUEST_PROTOCOL_VERSION, 1))
	sendPacket(writer, makeCommandLong(ardupilotmega.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_EXTENDED_STATUS, 2))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_POSITION, 2))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_EXTRA1, 4))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_EXTRA2, 4))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_EXTRA3, 4))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_RAW_SENSORS, 2))
	sendPacket(writer, makeRequestDataStream(ardupilotmega.MAV_DATA_STREAM_RC_CHANNELS, 2))
	sendPacket(writer, makeHeartbeat())
	sendPacket(writer, makeStatustext("Mission Planner 1.3.74"))
	sendPacket(writer, makeCommandLong(ardupilotmega.MAV_CMD_DO_SEND_BANNER, 0))
	sendPacket(writer, makeFileTransferProtocol([]byte{0, 0, 0, 4, 16, 0, 0, 0, 0, 0, 0, 0, 64, 80, 65, 82, 65, 77, 47, 112, 97, 114, 97, 109, 46, 112, 99, 107, 0}))
}
