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
	bytes := []byte{
		magicNumber,
		byte(len(p.Payload)),
		p.SeqID,
		p.SysID,
		p.CompID,
		uint8(p.MsgID),
	} // header
	bytes = append(bytes, p.Payload...)
	bytes = append(bytes, u16ToBytes(p.Checksum)...)
	return bytes
}

func (p *Packet) fixChecksum(dialect *Dialect) error {
	crc := NewX25()
	crc.WriteByte(byte(len(p.Payload)))
	crc.WriteByte(p.SeqID)
	crc.WriteByte(p.SysID)
	crc.WriteByte(p.CompID)
	crc.WriteByte(byte(p.MsgID >> 0))
	crc.Write(p.Payload)
	crcx, ok := dialect.crcExtras[p.MsgID]
	if !ok {
		return ErrUnknownMsgID
	}
	crc.WriteByte(crcx)
	p.Checksum = crc.Sum16()
	return nil
}
