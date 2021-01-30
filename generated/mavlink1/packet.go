/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import (
	"fmt"
)

var (
	msgConstructors = map[MessageID]func(*Packet) Message{}
	msgNames        = map[MessageID]string{}
)

// Packet is a wire type for encoding/decoding mavlink messages.
// use the ToPacket() and FromPacket() routines on specific message
// types to convert them to/from the Message type.
type Packet struct {
	SeqID    uint8     // Sequence of packet
	SysID    uint8     // ID of message sender system/aircraft
	CompID   uint8     // ID of the message sender component
	MsgID    MessageID // ID of message in payload
	Payload  []byte
	Checksum uint16
}

func (p *Packet) nextSeqNum() byte {
	currentSeqNum++
	return currentSeqNum
}

// Encode trying to encode message to packet
func (p *Packet) encode(sysID, compID uint8, m Message) error {
	p.SeqID = p.nextSeqNum()
	p.SysID = sysID
	p.CompID = compID
	return p.Encode(m)
}

// Encode trying to encode message to packet
func (p *Packet) Encode(m Message) error {
	if err := m.Pack(p); err != nil {
		return err
	}
	if err := p.fixChecksum(msgCrcExtras[m.MsgID()]); err != nil {
		return err
	}
	return nil
}

// Decode trying to decode message to packet
func (p *Packet) Decode(m Message) error {
	return m.Unpack(p)
}

// Unmarshal trying to de-serialize byte slice to packet
func Unmarshal(buffer []byte, p *Packet) error {
	parser := parsersPool.Get().(*Parser)
	defer parsersPool.Put(parser)
	for _, c := range buffer {
		packet, err := parser.parseChar(c)
		if err != nil {
			return err
		}
		if packet != nil {
			*p = *packet
			return nil
		}
	}
	return ErrNoNewData
}

// Marshal trying to serialize byte slice from packet
func Marshal(p *Packet) ([]byte, error) {
	if p == nil {
		return nil, ErrNilPointerReference
	}
	return p.Bytes(), nil
}

// Bytes make byte slice from packet
func (p *Packet) Bytes() []byte {
	bytes := make([]byte, 0, 8+len(p.Payload))
	// header
	bytes = append(bytes,
		magicNumber,
		byte(len(p.Payload)),
		p.SeqID,
		p.SysID,
		p.CompID,
		uint8(p.MsgID),
	)
	// payload
	bytes = append(bytes, p.Payload...)
	// crc
	bytes = append(bytes, p.u16ToBytes(p.Checksum)...)
	return bytes
}

func (p *Packet) fixChecksum(crcExtra uint8) error {
	crc := NewX25()
	crc.WriteByte(byte(len(p.Payload)))
	crc.WriteByte(p.SeqID)
	crc.WriteByte(p.SysID)
	crc.WriteByte(p.CompID)
	crc.WriteByte(byte(p.MsgID >> 0))
	crc.Write(p.Payload)
	crc.WriteByte(crcExtra)
	p.Checksum = crc.Sum16()
	return nil
}

func (p *Packet) u16ToBytes(v uint16) []byte {
	return []byte{byte(v & 0xff), byte(v >> 8)}
}

// Message function produce message from packet
func (p *Packet) Message() (Message, error) {
	constructor, ok := msgConstructors[p.MsgID]
	if !ok {
		return nil, ErrUnknownMsgID
	}
	return constructor(p), nil
}

// String function return string view of Packet struct
func (p *Packet) String() string {
	return fmt.Sprintf(
		"&mavlink.Packet{ SeqID: %d, SysID: %d, CompID: %d, MsgID: %d, Payload: %s, Checksum: %d }",
		p.SeqID,
		p.SysID,
		p.CompID,
		int64(p.MsgID),
		func() string {
			msg, err := p.Message()
			if err != nil {
				return fmt.Sprintf("%0X", p.Payload)
			}
			return msg.String()
		}(),
		p.Checksum,
	)
}
