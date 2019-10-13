/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import "sync"

type Parser struct {
	state  MAVLINK_PARSE_STATE
	packet Packet
	crc    *X25
}

var parsersPool = &sync.Pool{
	New: func() interface{} {
		return new(Parser)
	},
}

func (p *Parser) Reset() {
	p.state = MAVLINK_PARSE_STATE_UNINIT
	p.crc.Reset()
	p.crc = nil
}

func (parser *Parser) parseChar(c byte) (*Packet, error) {
	switch parser.state {
	case MAVLINK_PARSE_STATE_UNINIT,
		MAVLINK_PARSE_STATE_IDLE,
		MAVLINK_PARSE_STATE_GOT_BAD_CRC,
		MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE:
		if c == magicNumber {
			parser.crc = NewX25()
			parser.state = MAVLINK_PARSE_STATE_GOT_STX
		}
	case MAVLINK_PARSE_STATE_GOT_STX:
		parser.packet.Payload = make([]byte, 0, c)
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_LENGTH
	case MAVLINK_PARSE_STATE_GOT_LENGTH:
		parser.packet.InCompatFlags = c
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS
	case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
		parser.packet.CompatFlags = c
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS
	case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
		parser.packet.SeqID = c
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_SEQ
	case MAVLINK_PARSE_STATE_GOT_SEQ:
		parser.packet.SysID = c
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_SYSID
	case MAVLINK_PARSE_STATE_GOT_SYSID:
		parser.packet.CompID = c
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_COMPID
	case MAVLINK_PARSE_STATE_GOT_COMPID:
		parser.packet.MsgID = MessageID(c)
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_MSGID1
	case MAVLINK_PARSE_STATE_GOT_MSGID1:
		parser.packet.MsgID += MessageID(c) << 8
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_MSGID2
	case MAVLINK_PARSE_STATE_GOT_MSGID2:
		parser.packet.MsgID += MessageID(c) << 8 * 2
		parser.crc.WriteByte(c)
		parser.state = MAVLINK_PARSE_STATE_GOT_MSGID3
	case MAVLINK_PARSE_STATE_GOT_MSGID3:
		parser.packet.Payload = append(parser.packet.Payload, c)
		parser.crc.WriteByte(c)
		if len(parser.packet.Payload) == cap(parser.packet.Payload) {
			parser.state = MAVLINK_PARSE_STATE_GOT_PAYLOAD
		}
	case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
		crcExtra, err := dialects.findCrcX(parser.packet.MsgID)
		if err != nil {
			crcExtra = 0
		}
		parser.crc.WriteByte(crcExtra)
		if c != uint8(parser.crc.Sum16()&0xFF) {
			parser.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC
			parser.packet = Packet{}
			return nil, ErrCrcFail
		}
		parser.state = MAVLINK_PARSE_STATE_GOT_CRC1
	case MAVLINK_PARSE_STATE_GOT_CRC1:
		if c == uint8(parser.crc.Sum16()>>8) {
			parser.packet.Checksum = parser.crc.Sum16()
			parser.state = MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE
			result := parser.packet
			parser.packet = Packet{}
			return &result, nil
		}
		parser.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC
		parser.packet = Packet{}
		return nil, ErrCrcFail
	}
	return nil, nil
}
