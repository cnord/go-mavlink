/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// packetTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func packetTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"// Packet is a wire type for encoding/decoding mavlink messages.\n" +
		"// use the ToPacket() and FromPacket() routines on specific message\n" +
		"// types to convert them to/from the Message type.\n" +
		"type Packet struct {\n" +
		"{{- if .Mavlink2}}\n" +
		"\tInCompatFlags uint8     // incompat flags\n" +
		"\tCompatFlags   uint8     // compat flags\n" +
		"{{- end}}\n" +
		"\tSeqID         uint8     // Sequence of packet\n" +
		"\tSysID         uint8     // ID of message sender system/aircraft\n" +
		"\tCompID        uint8     // ID of the message sender component\n" +
		"\tMsgID         MessageID // ID of message in payload\n" +
		"\tPayload       []byte\n" +
		"\tChecksum      uint16\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) nextSeqNum() byte {\n" +
		"\tcurrentSeqNum++\n" +
		"\treturn currentSeqNum\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) Encode(sysID, compID uint8, m Message) error {\n" +
		"\tp.SeqID = p.nextSeqNum()\n" +
		"\tp.SysID = sysID\n" +
		"\tp.CompID = compID\n" +
		"\tif err := m.Pack(p); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\tif err := p.fixChecksum(m.Dialect()); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) EncodeMessage(m Message) error {\n" +
		"\tif err := m.Pack(p); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\tif err := p.fixChecksum(m.Dialect()); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) Bytes() []byte {\n" +
		"    bytes := make([]byte, 0, {{if .Mavlink2 -}} 12 {{- else -}} 8 {{- end}}+len(p.Payload))\n" +
		"    // header\n" +
		"    bytes = append(bytes,\n" +
		"\t    magicNumber,\n" +
		"\t    byte(len(p.Payload)),\n" +
		"{{- if .Mavlink2}}\n" +
		"\t    uint8(p.InCompatFlags),\n" +
		"\t    uint8(p.CompatFlags),\n" +
		"{{- end}}\n" +
		"\t    p.SeqID,\n" +
		"\t    p.SysID,\n" +
		"\t    p.CompID,\n" +
		"\t    uint8(p.MsgID),\n" +
		"{{- if .Mavlink2}}\n" +
		"\t    uint8(p.MsgID >> 8),\n" +
		"\t    uint8(p.MsgID >> 16),\n" +
		"{{- end}}\n" +
		"    )\n" +
		"    // payload\n" +
		"\tbytes = append(bytes, p.Payload...)\n" +
		"\t// crc\n" +
		"\tbytes = append(bytes, u16ToBytes(p.Checksum)...)\n" +
		"\treturn bytes\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) fixChecksum(dialect *Dialect) error {\n" +
		"\tcrc := NewX25()\n" +
		"\tcrc.WriteByte(byte(len(p.Payload)))\n" +
		"{{- if .Mavlink2}}\n" +
		"\tcrc.WriteByte(p.InCompatFlags)\n" +
		"\tcrc.WriteByte(p.CompatFlags)\n" +
		"{{- end}}\n" +
		"\tcrc.WriteByte(p.SeqID)\n" +
		"\tcrc.WriteByte(p.SysID)\n" +
		"\tcrc.WriteByte(p.CompID)\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 0 ))\n" +
		"{{- if .Mavlink2}}\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 8 ))\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 16))\n" +
		"{{- end}}\n" +
		"\tcrc.Write(p.Payload)\n" +
		"\tcrcx, ok := dialect.crcExtras[p.MsgID]\n" +
		"\tif !ok {\n" +
		"\t\treturn ErrUnknownMsgID\n" +
		"\t}\n" +
		"\tcrc.WriteByte(crcx)\n" +
		"\tp.Checksum = crc.Sum16()\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		""
	return tmpl
}
