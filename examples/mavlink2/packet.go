/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

// Packet is a wire type for encoding/decoding mavlink messages.
// use the ToPacket() and FromPacket() routines on specific message
// types to convert them to/from the Message type.
type Packet struct {
	InCompatFlags uint8     // incompat flags
	CompatFlags   uint8     // compat flags
	SeqID         uint8     // Sequence of packet
	SysID         uint8     // ID of message sender system/aircraft
	CompID        uint8     // ID of the message sender component
	MsgID         MessageID // ID of message in payload
	Payload       []byte
	Checksum      uint16
}

func (p *Packet) nextSeqNum() byte {
	currentSeqNum++
	return currentSeqNum
}

func (p *Packet) Encode(sysID, compID uint8, m Message) error {
	p.SeqID = p.nextSeqNum()
	p.SysID = sysID
	p.CompID = compID
	if err := m.Pack(p); err != nil {
		return err
	}
	if err := p.fixChecksum(m.Dialect()); err != nil {
		return err
	}
	return nil
}

func (p *Packet) EncodeMessage(m Message) error {
	if err := m.Pack(p); err != nil {
		return err
	}
	if err := p.fixChecksum(m.Dialect()); err != nil {
		return err
	}
	return nil
}

func (p *Packet) Bytes() []byte {
	bytes := make([]byte, 0, 12+len(p.Payload))
	// header
	bytes = append(bytes,
		magicNumber,
		byte(len(p.Payload)),
		uint8(p.InCompatFlags),
		uint8(p.CompatFlags),
		p.SeqID,
		p.SysID,
		p.CompID,
		uint8(p.MsgID),
		uint8(p.MsgID>>8),
		uint8(p.MsgID>>16),
	)
	// payload
	bytes = append(bytes, p.Payload...)
	// crc
	bytes = append(bytes, u16ToBytes(p.Checksum)...)
	return bytes
}

func (p *Packet) fixChecksum(dialect *Dialect) error {
	crc := NewX25()
	crc.WriteByte(byte(len(p.Payload)))
	crc.WriteByte(p.InCompatFlags)
	crc.WriteByte(p.CompatFlags)
	crc.WriteByte(p.SeqID)
	crc.WriteByte(p.SysID)
	crc.WriteByte(p.CompID)
	crc.WriteByte(byte(p.MsgID >> 0))
	crc.WriteByte(byte(p.MsgID >> 8))
	crc.WriteByte(byte(p.MsgID >> 16))
	crc.Write(p.Payload)
	crcx, ok := dialect.crcExtras[p.MsgID]
	if !ok {
		return ErrUnknownMsgID
	}
	crc.WriteByte(crcx)
	p.Checksum = crc.Sum16()
	return nil
}
