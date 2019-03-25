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
		"func (p *Parser) Reset() {\n" +
		"\tp.state = MAVLINK_PARSE_STATE_UNINIT\n" +
		"\tp.crc.Reset()\n" +
		"\tp.crc = nil\n" +
		"}\n" +
		"\n" +
		"func (parser *Parser) parseChar(c byte) (*Packet, error) {\n" +
		"\tswitch parser.state {\n" +
		"\tcase MAVLINK_PARSE_STATE_UNINIT,\n" +
		"\t\t MAVLINK_PARSE_STATE_IDLE,\n" +
		"\t\t MAVLINK_PARSE_STATE_GOT_BAD_CRC,\n" +
		"\t\t MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE :\n" +
		"\t\tif c == magicNumber {\n" +
		"\t\t\tparser.crc = NewX25()\n" +
		"\t\t\tparser.state = MAVLINK_PARSE_STATE_GOT_STX\n" +
		"\t\t}\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_STX:\n" +
		"\t\tparser.packet.Payload = make([]byte, 0, c)\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_LENGTH\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_LENGTH:\n" +
		"{{- if .Mavlink2}}\n" +
		"\t\tparser.packet.InCompatFlags = c\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:\n" +
		"\t\tparser.packet.CompatFlags = c\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:\n" +
		"{{- end}}\n" +
		"\t\tparser.packet.SeqID = c\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_SEQ\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_SEQ:\n" +
		"\t\tparser.packet.SysID = c\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_SYSID\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_SYSID:\n" +
		"\t\tparser.packet.CompID = c\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_COMPID\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_COMPID:\n" +
		"\t\tparser.packet.MsgID = MessageID(c)\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_MSGID1\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID1:\n" +
		"{{- if .Mavlink2}}\n" +
		"\t\tparser.packet.MsgID += MessageID(c << 8)\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_MSGID2\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID2:\n" +
		"\t\tparser.packet.MsgID += MessageID(c << 8 * 2)\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tparser.state = MAVLINK_PARSE_STATE_GOT_MSGID3\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_MSGID3:\n" +
		"{{- end}}\n" +
		"\t\tparser.packet.Payload = append(parser.packet.Payload, c)\n" +
		"\t\tparser.crc.WriteByte(c)\n" +
		"\t\tif len(parser.packet.Payload) == cap(parser.packet.Payload) {\n" +
		"\t\t\tparser.state = MAVLINK_PARSE_STATE_GOT_PAYLOAD\n" +
		"\t\t}\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_PAYLOAD:\n" +
		"\t\tcrcExtra, err := dialects.findCrcX(parser.packet.MsgID)\n" +
		"\t\tif err != nil {\n" +
		"\t\t\tcrcExtra = 0\n" +
		"\t\t}\n" +
		"\t\tparser.crc.WriteByte(crcExtra)\n" +
		"\t\tif c != uint8(parser.crc.Sum16()&0xFF) {\n" +
		"\t\t\tparser.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC\n" +
		"\t\t\tparser.packet = Packet{}\n" +
		"\t\t\treturn nil, ErrCrcFail\n" +
		"\t\t}\n" +
		"        parser.state = MAVLINK_PARSE_STATE_GOT_CRC1\n" +
		"\tcase MAVLINK_PARSE_STATE_GOT_CRC1:\n" +
		"\t\tif c == uint8(parser.crc.Sum16()>>8) {\n" +
		"\t\t\tparser.packet.Checksum = parser.crc.Sum16()\n" +
		"\t\t\tparser.state = MAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE\n" +
		"\t\t\tresult := parser.packet\n" +
		"\t\t\tparser.packet = Packet{}\n" +
		"\t\t\treturn &result, nil\n" +
		"\t\t}\n" +
		"        parser.state = MAVLINK_PARSE_STATE_GOT_BAD_CRC\n" +
		"        parser.packet = Packet{}\n" +
		"        return nil, ErrCrcFail\n" +
		"\t}\n" +
		"\treturn nil, nil\n" +
		"}\n" +
		"\n" +
		""
	return tmpl
}
