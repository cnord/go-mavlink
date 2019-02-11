package mavlink

import (
	"bufio"
	"errors"
	"io"
	"sync"

	"github.com/cnord/go-mavlink/x25"
)

//go:generate mavgen -f ../mavlink-upstream/message_definitions/v1.0/common.xml
//go:generate mavgen -f ../mavlink-upstream/message_definitions/v1.0/ardupilotmega.xml
//go:generate mavgen -f ../mavlink-upstream/message_definitions/v1.0/ASLUAV.xml
//go:generate mavgen -f ../mavlink-upstream/message_definitions/v1.0/matrixpilot.xml
//go:generate mavgen -f ../mavlink-upstream/message_definitions/v1.0/ualberta.xml


const (
	ctxMavlink1      = 0xfe
	ctxMavlink2      = 0xfd
	numChecksumBytes = 2
	hdrLenMavlink1   = 6
	hdrLenMavlink2   = 10
)

var (
	ErrUnknownMsgID = errors.New("unknown msg id")
	ErrCrcFail      = errors.New("checksum did not match")
)

type MessageID uint16

// basic type for encoding/decoding mavlink messages.
// use the Pack() and Unpack() routines on specific message
// types to convert them to/from the Packet type.
type Message interface {
	Pack(*Packet) error
	Unpack(*Packet) error
	MsgID() MessageID
	MsgName() string
}

// wire type for encoding/decoding mavlink messages.
// use the ToPacket() and FromPacket() routines on specific message
// types to convert them to/from the Message type.
type Packet struct {
	InCompatFlags uint8	  // incompat flags
	CompatFlags   uint8	  // compat flags
	SeqID         uint8     // Sequence of packet
	SysID     	  uint8     // ID of message sender system/aircraft
	CompID    	  uint8     // ID of the message sender component
	DialectID 	  uint8
	MsgID     	  MessageID // ID of message in payload
	Payload   	  []byte
	Checksum  	  uint16
}

type Decoder struct {
	sync.Mutex
	CurrSeqID uint8        // last seq id decoded
	Dialects  DialectSlice // dialects that can be decoded
	br        *bufio.Reader
	buffer    []byte // stores bytes we've read from br
}

type Encoder struct {
	sync.Mutex
	CurrSeqID uint8        // last seq id encoded
	Dialects  DialectSlice // dialects that can be encoded
	bw        *bufio.Writer
}

func NewDecoder(r io.Reader) *Decoder {
	d := &Decoder{
		Dialects: DialectSlice{DialectDefault},
	}

	if v, ok := r.(*bufio.Reader); ok {
		d.br = v
	} else {
		d.br = bufio.NewReader(r)
	}

	return d
}

func NewEncoder(w io.Writer) *Encoder {

	e := &Encoder{
		Dialects: DialectSlice{DialectDefault},
	}

	if v, ok := w.(*bufio.Writer); ok {
		e.bw = v
	} else {
		e.bw = bufio.NewWriter(w)
	}

	return e
}

// helper to create packet w/header populated with received bytes
func newPacketFromBytes(b []byte, mavlinkVersion int) (*Packet, int) {
	if(mavlinkVersion == 2){
		return &Packet{
			InCompatFlags: b[1],
			CompatFlags: b[2],
			SeqID:  b[2],
			SysID:  b[4],
			CompID: b[5],
			DialectID: uint8(0), // mavgen.py ignore this field
			MsgID:  MessageID(b[6] + (b[7] << 8)),
		}, int(b[0])
	}else if(mavlinkVersion == 1){
		return &Packet{
			InCompatFlags: uint8(0),
			CompatFlags: uint8(0),
			SeqID:  b[1],
			SysID:  b[2],
			CompID: b[3],
			DialectID: uint8(0),
			MsgID:  MessageID(b[4]),
		}, int(b[0])
	}else{
		return nil, 0
	}
}

// Decoder reads and parses from its reader
// Typically, the caller will check the p.MsgID to see if it's
// a message they're interested in, and convert it to the
// corresponding type via Message.FromPacket()
func (dec *Decoder) Decode() (*Packet, error) {
	for {
		startFoundInBuffer := false
		var (
			mavlinkVersion 	int
			hdrLen 			int
		)
		// discard bytes in buffer before start byte
		for i, b := range dec.buffer {
			if b == ctxMavlink1 {
				dec.buffer = dec.buffer[i:]
				startFoundInBuffer = true
				mavlinkVersion = 1
				hdrLen = hdrLenMavlink1
				break
			}else if b == ctxMavlink2 {
				dec.buffer = dec.buffer[i:]
				startFoundInBuffer = true
				mavlinkVersion = 2
				hdrLen = hdrLenMavlink2
				break
			}
		}

		// if start not found, read until we see start byte
		if !startFoundInBuffer {
			for {
				c, err := dec.br.ReadByte()
				if err != nil {
					return nil, err
				}
				if c == ctxMavlink1 {
					dec.buffer = append(dec.buffer, c)
					mavlinkVersion = 1
					hdrLen = hdrLenMavlink1
					break
				}else if c == ctxMavlink2 {
					dec.buffer = append(dec.buffer, c)
					mavlinkVersion = 2
					hdrLen = hdrLenMavlink2
					break
				}
			}
		}

		if len(dec.buffer) < 2 {
			// read length byte
			bytesRead := make([]byte, 1)
			n, err := io.ReadAtLeast(dec.br, bytesRead, 1)
			if err != nil {
				return nil, err
			}

			dec.buffer = append(dec.buffer, bytesRead[:n]...)
		}

		// buffer[1] is LENGTH and we've already read len(buffer) bytes
		payloadLen := int(dec.buffer[1])
		packetLen := hdrLen + payloadLen + numChecksumBytes
		bytesNeeded := packetLen - len(dec.buffer)
		if bytesNeeded > 0 {
			bytesRead := make([]byte, bytesNeeded)
			n, err := io.ReadAtLeast(dec.br, bytesRead, bytesNeeded)
			if err != nil {
				return nil, err
			}
			dec.buffer = append(dec.buffer, bytesRead[:n]...)
		}

		// hdr contains LENGTH, SEQ, SYSID, COMPID, MSGID
		// (hdrLen - 1) because we don't include the start byte
		hdr := make([]byte, hdrLen-1)
		// don't include start byte
		hdr = dec.buffer[1:hdrLen]

		p, payloadLen := newPacketFromBytes(hdr, mavlinkVersion)

		crc := x25.New()
		crc.Write(hdr)

		payloadStart := hdrLen
		p.Payload = dec.buffer[payloadStart : payloadStart+payloadLen]
		crc.Write(p.Payload)

		crcx, err := dec.Dialects.findCrcX(p.MsgID)
		if err != nil {
			dec.buffer = dec.buffer[1:]
			// return error here to allow caller to decide if stream is
			// corrupted or if we're getting the wrong dialect
			return p, err
		}
		crc.WriteByte(crcx)

		p.Checksum = bytesToU16(dec.buffer[payloadStart+payloadLen : payloadStart+payloadLen+numChecksumBytes])

		// does the transmitted checksum match our computed checksum?
		if p.Checksum != crc.Sum16() {
			// strip off start byte
			dec.buffer = dec.buffer[1:]
		} else {
			dec.CurrSeqID = p.SeqID
			dec.buffer = dec.buffer[packetLen:]
			return p, nil
		}
	}
}

// Decode a packet from a previously received buffer (such as a UDP packet),
// b must contain a complete message
func (dec *Decoder) DecodeBytes(b []byte, mavlinkVersion int) (*Packet, error) {
	var(
		hdrLen int
		startByte byte
	)
	if(mavlinkVersion == 2){
		hdrLen = hdrLenMavlink2
		startByte = ctxMavlink2
	}else if(mavlinkVersion == 1){
		hdrLen = hdrLenMavlink1
		startByte = ctxMavlink1
	}

	if len(b) < hdrLen || b[0] != startByte {
		return nil, errors.New("invalid header")
	}

	p, payloadLen := newPacketFromBytes(b[1:], mavlinkVersion)

	crc := x25.New()
	p.Payload = b[hdrLen : hdrLen+payloadLen]
	crc.Write(b[1 : hdrLen+payloadLen])

	crcx, err := dec.Dialects.findCrcX(p.MsgID)
	if err != nil {
		return p, err
	}
	crc.WriteByte(crcx)

	p.Checksum = bytesToU16(b[hdrLen+payloadLen:])

	// does the transmitted checksum match our computed checksum?
	if p.Checksum != crc.Sum16() {
		return p, ErrCrcFail
	}

	dec.CurrSeqID = p.SeqID
	return p, nil
}

// helper that accepts a Message, internally converts it to a Packet,
// sets the Packet's SeqID based on the
// and then writes it to its writer via EncodePacket()
func (enc *Encoder) Encode(sysID, compID uint8, m Message, mavlinkVersion int) error {
	var p Packet
	if err := m.Pack(&p); err != nil {
		return err
	}

	p.SysID, p.CompID = sysID, compID

	return enc.EncodePacket(&p, mavlinkVersion)
}

// Encode writes p to its writer
func (enc *Encoder) EncodePacket(p *Packet, mavlinkVersion int) error {
	var(
		hdr []byte
	)
	if(mavlinkVersion == 2){
		hdr = []byte{ctxMavlink2, uint8(0), uint8(0), byte(len(p.Payload)), enc.CurrSeqID, p.SysID, p.CompID, uint8(0), uint8(p.MsgID & 0xFF), uint8((p.MsgID >> 8) & 0xFF)}
	}else if(mavlinkVersion == 1){
		hdr = []byte{ctxMavlink1, byte(len(p.Payload)), enc.CurrSeqID, p.SysID, p.CompID, uint8(p.MsgID)}
	}

	// header
	if err := enc.writeAndCheck(hdr); err != nil {
		return err
	}

	crc := x25.New()

	crc.Write(hdr[1:]) // don't include start byte

	// payload
	if err := enc.writeAndCheck(p.Payload); err != nil {
		return err
	}
	crc.Write(p.Payload)

	// crc extra
	crcx, err := enc.Dialects.findCrcX(p.MsgID)
	if err != nil {
		return err
	}
	crc.WriteByte(crcx)

	// crc
	crcBytes := u16ToBytes(crc.Sum16())
	if err := enc.writeAndCheck(crcBytes); err != nil {
		return err
	}

	err = enc.bw.Flush()
	if err == nil {
		enc.CurrSeqID++
	}

	return err
}

// helper to check both the write and writelen status
func (enc *Encoder) writeAndCheck(p []byte) error {
	n, err := enc.bw.Write(p)
	if err == nil && n != len(p) {
		return io.ErrShortWrite
	}

	return err
}

func u16ToBytes(v uint16) []byte {
	return []byte{byte(v & 0xff), byte(v >> 8)}
}

func bytesToU16(p []byte) uint16 {
	// NB: does not check size of p
	return (uint16(p[1]) << 8) | uint16(p[0])
}
