/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// parserTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func parserTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import \"sync\"\n" +
		"\n" +
		"// Parser is a state machine which parse bytes to mavlink.Packet\n" +
		"type Parser struct {\n" +
		"\tstate  MAVLINK_PARSE_STATE\n" +
		"\tpacket Packet\n" +
		"\tcrc    *X25\n" +
		"}\n" +
		"\n" +
		"var parsersPool = &sync.Pool{\n" +
		"\tNew: func() interface{} {\n" +
		"\t\treturn new(Parser)\n" +
		"\t},\n" +
		"}\n" +
		"\n" +
		"// Reset set parser to idle state\n" +
		"func (p *Parser) Reset() {\n" +
		"\tp.state = MAVLINK_PARSE_STATE_UNINIT\n" +
		"\tp.crc.Reset()\n" +
		"\tp.crc = nil\n" +
		"}\n" +
		"\n" +
		"func (p *Parser) parseChar(c byte) (*Packet, error) {\n" +
		"\tswitch p.state {\n" +
		"\tcase MAVLINK_PARSE_STATE_UNINIT,\n" +
		"\t\t MAVLINK_PARSE_STATE_IDLE,\n" +
		"\t\t MAVLINK_PARSE_STATE_GOT_BAD_CRC,\n" +
		"\t\t MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE :\n" +
		"\t\tif c == magicNumber {\n" +
		"\t\t\tp.crc = NewX25()\n" +
		"\t\t\tp.state = MAVLINK_PARSE_STATE_GOT_STX\n" +
		"\t\t}\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_STX:\n" +
		"\t\tp.packet.Payload = make([]byte, 0, c)\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_LENGTH\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_LENGTH:\n" +
		"{{- if .Mavlink2}}\n" +
		"\t\tp.packet.InCompatFlags = c\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:\n" +
		"\t\tp.packet.CompatFlags = c\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:\n" +
		"{{- end}}\n" +
		"\t\tp.packet.SeqID = c\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_SEQ\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_SEQ:\n" +
		"\t\tp.packet.SysID = c\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_SYSID\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_SYSID:\n" +
		"\t\tp.packet.CompID = c\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_COMPID\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_COMPID:\n" +
		"\t\tp.packet.MsgID = MessageID(c)\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_MSGID1\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID1:\n" +
		"{{- if .Mavlink2}}\n" +
		"\t\tp.packet.MsgID += MessageID(c) << 8\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_MSGID2\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID2:\n" +
		"\t\tp.packet.MsgID += MessageID(c) << 8 * 2\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tp.state = MAVLINK_PARSE_STATE_GOT_MSGID3\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID3:\n" +
		"{{- end}}\n" +
		"\t\tp.packet.Payload = append(p.packet.Payload, c)\n" +
		"\t\tp.crc.WriteByte(c)\n" +
		"\t\tif len(p.packet.Payload) == cap(p.packet.Payload) {\n" +
		"\t\t\tp.state = MAVLINK_PARSE_STATE_GOT_PAYLOAD\n" +
		"\t\t}\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_PAYLOAD:\n" +
		"\t\tcrcExtra, err := dialects.findCrcX(p.packet.MsgID)\n" +
		"\t\tif err != nil {\n" +
		"\t\t\tcrcExtra = 0\n" +
		"\t\t}\n" +
		"\t\tp.crc.WriteByte(crcExtra)\n" +
		"\t\tif c != uint8(p.crc.Sum16()&0xFF) {\n" +
		"\t\t\tp.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC\n" +
		"\t\t\tp.packet = Packet{}\n" +
		"\t\t\treturn nil, ErrCrcFail\n" +
		"\t\t}\n" +
		"        p.state = MAVLINK_PARSE_STATE_GOT_CRC1\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_CRC1:\n" +
		"\t\tif c == uint8(p.crc.Sum16()>>8) {\n" +
		"\t\t\tp.packet.Checksum = p.crc.Sum16()\n" +
		"\t\t\tp.state = MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE\n" +
		"\t\t\tresult := p.packet\n" +
		"\t\t\tp.packet = Packet{}\n" +
		"\t\t\treturn &result, nil\n" +
		"\t\t}\n" +
		"        p.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC\n" +
		"        p.packet = Packet{}\n" +
		"        return nil, ErrCrcFail\n" +
		"\t}\n" +
		"\treturn nil, nil\n" +
		"}\n" +
		"\n" +
		""
	return tmpl
}
