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
		"var (\n" +
		"    msgConstructors = map[MessageID]func(*Packet) Message{}\n" +
		"    msgNames = map[MessageID]string{}\n" +
		")\n" +
		"\n" +
		"// Packet is a wire type for encoding/decoding mavlink messages.\n" +
		"// use the ToPacket() and FromPacket() routines on specific message\n" +
		"// types to convert them to/from the Message type.\n" +
		"type Packet struct {\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
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
		"// Encode trying to encode message to packet\n" +
		"func (p *Packet) encode(sysID, compID uint8, m Message) error {\n" +
		"\tp.SeqID = p.nextSeqNum()\n" +
		"\tp.SysID = sysID\n" +
		"\tp.CompID = compID\n" +
		"\treturn p.Encode(m)\n" +
		"}\n" +
		"\n" +
		"// Encode trying to encode message to packet\n" +
		"func (p *Packet) Encode(m Message) error {\n" +
		"\tif err := m.Pack(p); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\tif err := p.fixChecksum(msgCrcExtras[m.MsgID()]); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		"// Decode trying to decode message to packet\n" +
		"func (p *Packet) Decode(m Message) error {\n" +
		"\treturn m.Unpack(p)\n" +
		"}\n" +
		"\n" +
		"// Unmarshal trying to de-serialize byte slice to packet\n" +
		"func Unmarshal(buffer []byte, p *Packet) error {\n" +
		"\tparser := parsersPool.Get().(*Parser)\n" +
		"\tdefer parsersPool.Put(parser)\n" +
		"\tfor _, c := range buffer {\n" +
		"\t\tpacket, err := parser.parseChar(c)\n" +
		"\t\tif err != nil {\n" +
		"\t\t\treturn err\n" +
		"\t\t}\n" +
		"\t\tif packet != nil {\n" +
		"\t\t\t*p = *packet\n" +
		"\t\t\treturn nil\n" +
		"\t\t}\n" +
		"\t}\n" +
		"\treturn ErrNoNewData\n" +
		"}\n" +
		"\n" +
		"// Marshal trying to serialize byte slice from packet\n" +
		"func Marshal(p *Packet) ([]byte, error) {\n" +
		"\tif p == nil {\n" +
		"\t\treturn nil, ErrNilPointerReference\n" +
		"\t}\n" +
		"\treturn p.Bytes(), nil\n" +
		"}\n" +
		"\n" +
		"// Bytes make byte slice from packet\n" +
		"func (p *Packet) Bytes() []byte {\n" +
		"    bytes := make([]byte, 0, {{if eq .MavlinkVersion 2 -}} 12 {{- else -}} 8 {{- end}}+len(p.Payload))\n" +
		"    // header\n" +
		"    bytes = append(bytes,\n" +
		"\t    magicNumber,\n" +
		"\t    byte(len(p.Payload)),\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
		"\t    uint8(p.InCompatFlags),\n" +
		"\t    uint8(p.CompatFlags),\n" +
		"{{- end}}\n" +
		"\t    p.SeqID,\n" +
		"\t    p.SysID,\n" +
		"\t    p.CompID,\n" +
		"\t    uint8(p.MsgID),\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
		"\t    uint8(p.MsgID >> 8),\n" +
		"\t    uint8(p.MsgID >> 16),\n" +
		"{{- end}}\n" +
		"    )\n" +
		"    // payload\n" +
		"\tbytes = append(bytes, p.Payload...)\n" +
		"\t// crc\n" +
		"\tbytes = append(bytes, p.u16ToBytes(p.Checksum)...)\n" +
		"\treturn bytes\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) fixChecksum(crcExtra uint8) error {\n" +
		"\tcrc := NewX25()\n" +
		"\tcrc.WriteByte(byte(len(p.Payload)))\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
		"\tcrc.WriteByte(p.InCompatFlags)\n" +
		"\tcrc.WriteByte(p.CompatFlags)\n" +
		"{{- end}}\n" +
		"\tcrc.WriteByte(p.SeqID)\n" +
		"\tcrc.WriteByte(p.SysID)\n" +
		"\tcrc.WriteByte(p.CompID)\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 0 ))\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 8 ))\n" +
		"\tcrc.WriteByte(byte(p.MsgID >> 16))\n" +
		"{{- end}}\n" +
		"\tcrc.Write(p.Payload)\n" +
		"\tcrc.WriteByte(crcExtra)\n" +
		"\tp.Checksum = crc.Sum16()\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		"func (p *Packet) u16ToBytes(v uint16) []byte {\n" +
		"\treturn []byte{byte(v & 0xff), byte(v >> 8)}\n" +
		"}\n" +
		"\n" +
		"// Message function produce message from packet\n" +
		"func (p *Packet) Message() (Message, error) {\n" +
		"\tconstructor, ok := msgConstructors[p.MsgID]\n" +
		"\tif !ok {\n" +
		"\t\treturn nil, ErrUnknownMsgID\n" +
		"\t}\n" +
		"\treturn constructor(p), nil\n" +
		"}\n" +
		""
	return tmpl
}
