/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// messageTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func messageTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import (\n" +
		"\t\"bufio\"\n" +
		"\t\"errors\"\n" +
		"\t\"io\"\n" +
		"\t\"sync\"\n" +
		"\t\"github.com/cnord/go-mavlink/x25\"\n" +
		")\n" +
		"\n" +
		"const (\n" +
		"\tnumChecksumBytes = 2\n" +
		"\tmagicNumber      = {{if .Mavlink2 -}} 0xfd {{- else -}} 0xfe {{- end}}\n" +
		"\thdrLen   \t\t = {{if .Mavlink2 -}} 10   {{- else -}} 6    {{- end}}\n" +
		")\n" +
		"\n" +
		"var (\n" +
		"\tErrUnknownMsgID = errors.New(\"unknown msg id\")\n" +
		"\tErrCrcFail      = errors.New(\"checksum did not match\")\n" +
		")\n" +
		"\n" +
		"type MessageID {{if .Mavlink2 -}} uint16 {{else}} uint8 {{- end}}\n" +
		"\n" +
		"// basic type for encoding/decoding mavlink messages.\n" +
		"// use the Pack() and Unpack() routines on specific message\n" +
		"// types to convert them to/from the Packet type.\n" +
		"type Message interface {\n" +
		"\tPack(*Packet) error\n" +
		"\tUnpack(*Packet) error\n" +
		"\tMsgID() MessageID\n" +
		"\tMsgName() string\n" +
		"}\n" +
		"\n" +
		"// wire type for encoding/decoding mavlink messages.\n" +
		"// use the ToPacket() and FromPacket() routines on specific message\n" +
		"// types to convert them to/from the Message type.\n" +
		"type Packet struct {\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\tInCompatFlags uint8\t  // incompat flags\n" +
		"\tCompatFlags   uint8\t  // compat flags\n" +
		"{{- end}}\n" +
		"\tSeqID         uint8     // Sequence of packet\n" +
		"\tSysID     \t  uint8     // ID of message sender system/aircraft\n" +
		"\tCompID    \t  uint8     // ID of the message sender component\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\tDialectID \t  uint8\n" +
		"{{- end}}\n" +
		"\tMsgID     \t  MessageID // ID of message in payload\n" +
		"\tPayload   \t  []byte\n" +
		"\tChecksum  \t  uint16\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\tSignature \t  []byte\n" +
		"{{- end}}\n" +
		"}\n" +
		"\n" +
		"type Decoder struct {\n" +
		"\tsync.Mutex\n" +
		"\tCurrSeqID uint8        // last seq id decoded\n" +
		"\tDialect   DialectSlice // dialects that can be decoded\n" +
		"\tbr        *bufio.Reader\n" +
		"\tbuffer    []byte // stores bytes we've read from br\n" +
		"}\n" +
		"\n" +
		"type Encoder struct {\n" +
		"\tsync.Mutex\n" +
		"\tCurrSeqID uint8        // last seq id encoded\n" +
		"\tDialects  DialectSlice // dialects that can be encoded\n" +
		"\tbw        *bufio.Writer\n" +
		"}\n" +
		"\n" +
		"func NewDecoder(r io.Reader) *Decoder {\n" +
		"\td := &Decoder{\n" +
		"\t\tDialects: DialectSlice{ {{- .DialectName -}} },\n" +
		"\t}\n" +
		"\n" +
		"\tif v, ok := r.(*bufio.Reader); ok {\n" +
		"\t\td.br = v\n" +
		"\t} else {\n" +
		"\t\td.br = bufio.NewReader(r)\n" +
		"\t}\n" +
		"\n" +
		"\treturn d\n" +
		"}\n" +
		"\n" +
		"func NewEncoder(w io.Writer) *Encoder {\n" +
		"\n" +
		"\te := &Encoder{\n" +
		"\t\tDialects: DialectSlice{ {{- .DialectName -}} },\n" +
		"\t}\n" +
		"\n" +
		"\tif v, ok := w.(*bufio.Writer); ok {\n" +
		"\t\te.bw = v\n" +
		"\t} else {\n" +
		"\t\te.bw = bufio.NewWriter(w)\n" +
		"\t}\n" +
		"\n" +
		"\treturn e\n" +
		"}\n" +
		"\n" +
		"// helper to create packet w/header populated with received bytes\n" +
		"func newPacketFromBytes(b []byte) (*Packet, int) {\n" +
		"\treturn &Packet{\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\t\tInCompatFlags: b[1],\n" +
		"\t\tCompatFlags: b[2],\n" +
		"{{- end -}}\n" +
		"\t\tSeqID:  b[ {{- if .Mavlink2 -}} 3 {{- else -}} 1 {{- end -}} ],\n" +
		"\t\tSysID:  b[ {{- if .Mavlink2 -}} 4 {{- else -}} 2 {{- end -}} ],\n" +
		"\t\tCompID: b[ {{- if .Mavlink2 -}} 5 {{- else -}} 2 {{- end -}} ],\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\t\tDialectID: uint8(0), // mavgen.py ignore this field\n" +
		"{{- end}}\n" +
		"\t\tMsgID:  MessageID( {{- if .Mavlink2 -}} b[6] + (b[7] << 8) {{- else -}} b[4] {{- end -}} ),\n" +
		"{{- if .Mavlink2 -}}\n" +
		"\t\tSignature: []byte{}, // mavgen.py ignore this field\n" +
		"{{- end}}\n" +
		"\t}, int(b[0])\n" +
		"}\n" +
		"\n" +
		"// Decoder reads and parses from its reader\n" +
		"// Typically, the caller will check the p.MsgID to see if it's\n" +
		"// a message they're interested in, and convert it to the\n" +
		"// corresponding type via Message.FromPacket()\n" +
		"func (dec *Decoder) Decode() (*Packet, error) {\n" +
		"\tfor {\n" +
		"\t\tstartFoundInBuffer := false\n" +
		"\t\t// discard bytes in buffer before start byte\n" +
		"\t\tfor i, b := range dec.buffer {\n" +
		"\t\t\tif b == magicNumber {\n" +
		"\t\t\t\tdec.buffer = dec.buffer[i:]\n" +
		"\t\t\t\tstartFoundInBuffer = true\n" +
		"\t\t\t\tbreak\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\n" +
		"\t\t// if start not found, read until we see start byte\n" +
		"\t\tif !startFoundInBuffer {\n" +
		"\t\t\tfor {\n" +
		"\t\t\t\tc, err := dec.br.ReadByte()\n" +
		"\t\t\t\tif err != nil {\n" +
		"\t\t\t\t\treturn nil, err\n" +
		"\t\t\t\t}\n" +
		"\t\t\t\tif c == magicNumber {\n" +
		"\t\t\t\t\tdec.buffer = append(dec.buffer, c)\n" +
		"\t\t\t\t\tbreak\n" +
		"\t\t\t\t}\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\n" +
		"\t\tif len(dec.buffer) < 2 {\n" +
		"\t\t\t// read length byte\n" +
		"\t\t\tbytesRead := make([]byte, 1)\n" +
		"\t\t\tn, err := io.ReadAtLeast(dec.br, bytesRead, 1)\n" +
		"\t\t\tif err != nil {\n" +
		"\t\t\t\treturn nil, err\n" +
		"\t\t\t}\n" +
		"\n" +
		"\t\t\tdec.buffer = append(dec.buffer, bytesRead[:n]...)\n" +
		"\t\t}\n" +
		"\n" +
		"\t\t// buffer[1] is LENGTH and we've already read len(buffer) bytes\n" +
		"\t\tpayloadLen := int(dec.buffer[1])\n" +
		"\t\tpacketLen := hdrLen + payloadLen + numChecksumBytes\n" +
		"\t\tbytesNeeded := packetLen - len(dec.buffer)\n" +
		"\t\tif bytesNeeded > 0 {\n" +
		"\t\t\tbytesRead := make([]byte, bytesNeeded)\n" +
		"\t\t\tn, err := io.ReadAtLeast(dec.br, bytesRead, bytesNeeded)\n" +
		"\t\t\tif err != nil {\n" +
		"\t\t\t\treturn nil, err\n" +
		"\t\t\t}\n" +
		"\t\t\tdec.buffer = append(dec.buffer, bytesRead[:n]...)\n" +
		"\t\t}\n" +
		"\n" +
		"\t\t// hdr contains LENGTH, SEQ, SYSID, COMPID, MSGID\n" +
		"\t\t// (hdrLen - 1) because we don't include the start byte\n" +
		"\t\thdr := make([]byte, hdrLen-1)\n" +
		"\t\t// don't include start byte\n" +
		"\t\thdr = dec.buffer[1:hdrLen]\n" +
		"\n" +
		"\t\tp, payloadLen := newPacketFromBytes(hdr, mavlinkVersion)\n" +
		"\n" +
		"\t\tcrc := x25.New()\n" +
		"\t\tcrc.Write(hdr)\n" +
		"\n" +
		"\t\tpayloadStart := hdrLen\n" +
		"\t\tp.Payload = dec.buffer[payloadStart : payloadStart+payloadLen]\n" +
		"\t\tcrc.Write(p.Payload)\n" +
		"\n" +
		"\t\tcrcx, err := dec.Dialects.findCrcX(p.MsgID)\n" +
		"\t\tif err != nil {\n" +
		"\t\t\tdec.buffer = dec.buffer[1:]\n" +
		"\t\t\t// return error here to allow caller to decide if stream is\n" +
		"\t\t\t// corrupted or if we're getting the wrong dialect\n" +
		"\t\t\treturn p, err\n" +
		"\t\t}\n" +
		"\t\tcrc.WriteByte(crcx)\n" +
		"\n" +
		"\t\tp.Checksum = bytesToU16(dec.buffer[payloadStart+payloadLen : payloadStart+payloadLen+numChecksumBytes])\n" +
		"\n" +
		"\t\t// does the transmitted checksum match our computed checksum?\n" +
		"\t\tif p.Checksum != crc.Sum16() {\n" +
		"\t\t\t// strip off start byte\n" +
		"\t\t\tdec.buffer = dec.buffer[1:]\n" +
		"\t\t} else {\n" +
		"\t\t\tdec.CurrSeqID = p.SeqID\n" +
		"\t\t\tdec.buffer = dec.buffer[packetLen:]\n" +
		"\t\t\treturn p, nil\n" +
		"\t\t}\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"// Decode a packet from a previously received buffer (such as a UDP packet),\n" +
		"// b must contain a complete message\n" +
		"func (dec *Decoder) DecodeBytes(b []byte) (*Packet, error) {\n" +
		"\tif len(b) < hdrLen || b[0] != startByte {\n" +
		"\t\treturn nil, errors.New(\"invalid header\")\n" +
		"\t}\n" +
		"\n" +
		"\tp, payloadLen := newPacketFromBytes(b[1:])\n" +
		"\n" +
		"\tcrc := x25.New()\n" +
		"\tp.Payload = b[hdrLen : hdrLen+payloadLen]\n" +
		"\tcrc.Write(b[1 : hdrLen+payloadLen])\n" +
		"\n" +
		"\tcrcx, err := dec.Dialects.findCrcX(p.MsgID)\n" +
		"\tif err != nil {\n" +
		"\t\treturn p, err\n" +
		"\t}\n" +
		"\tcrc.WriteByte(crcx)\n" +
		"\n" +
		"\tp.Checksum = bytesToU16(b[hdrLen+payloadLen:])\n" +
		"\n" +
		"\t// does the transmitted checksum match our computed checksum?\n" +
		"\tif p.Checksum != crc.Sum16() {\n" +
		"\t\treturn p, ErrCrcFail\n" +
		"\t}\n" +
		"\n" +
		"\tdec.CurrSeqID = p.SeqID\n" +
		"\treturn p, nil\n" +
		"}\n" +
		"\n" +
		"// helper that accepts a Message, internally converts it to a Packet,\n" +
		"// sets the Packet's SeqID based on the\n" +
		"// and then writes it to its writer via EncodePacket()\n" +
		"func (enc *Encoder) Encode(sysID, compID uint8, m Message) error {\n" +
		"\tvar p Packet\n" +
		"\tif err := m.Pack(&p); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\n" +
		"\tp.SysID, p.CompID = sysID, compID\n" +
		"\n" +
		"\treturn enc.EncodePacket(&p, mavlinkVersion)\n" +
		"}\n" +
		"\n" +
		"// Encode writes p to its writer\n" +
		"func (enc *Encoder) EncodePacket(p *Packet) error {\n" +
		"{{if .Mavlink2 -}}\n" +
		"\thdr := []byte{ctxMavlink2, uint8(0), uint8(0), byte(len(p.Payload)), enc.CurrSeqID, p.SysID, p.CompID, uint8(0), uint8(p.MsgID & 0xFF), uint8((p.MsgID >> 8) & 0xFF)}\n" +
		"{{- else -}}\n" +
		"\thdr = []byte{magicNumber, byte(len(p.Payload)), enc.CurrSeqID, p.SysID, p.CompID, uint8(p.MsgID)}\n" +
		"{{- end }}\n" +
		"\t// header\n" +
		"\tif err := enc.writeAndCheck(hdr); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\n" +
		"\tcrc := x25.New()\n" +
		"\n" +
		"\tcrc.Write(hdr[1:]) // don't include start byte\n" +
		"\n" +
		"\t// payload\n" +
		"\tif err := enc.writeAndCheck(p.Payload); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\tcrc.Write(p.Payload)\n" +
		"\n" +
		"\t// crc extra\n" +
		"\tcrcx, err := enc.Dialects.findCrcX(p.MsgID)\n" +
		"\tif err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\tcrc.WriteByte(crcx)\n" +
		"\n" +
		"\t// crc\n" +
		"\tcrcBytes := u16ToBytes(crc.Sum16())\n" +
		"\tif err := enc.writeAndCheck(crcBytes); err != nil {\n" +
		"\t\treturn err\n" +
		"\t}\n" +
		"\n" +
		"\terr = enc.bw.Flush()\n" +
		"\tif err == nil {\n" +
		"\t\tenc.CurrSeqID++\n" +
		"\t}\n" +
		"\n" +
		"\treturn err\n" +
		"}\n" +
		"\n" +
		"// helper to check both the write and writelen status\n" +
		"func (enc *Encoder) writeAndCheck(p []byte) error {\n" +
		"\tn, err := enc.bw.Write(p)\n" +
		"\tif err == nil && n != len(p) {\n" +
		"\t\treturn io.ErrShortWrite\n" +
		"\t}\n" +
		"\n" +
		"\treturn err\n" +
		"}\n" +
		"\n" +
		"func u16ToBytes(v uint16) []byte {\n" +
		"\treturn []byte{byte(v & 0xff), byte(v >> 8)}\n" +
		"}\n" +
		"\n" +
		"func bytesToU16(p []byte) uint16 {\n" +
		"\t// NB: does not check size of p\n" +
		"\treturn (uint16(p[1]) << 8) | uint16(p[0])\n" +
		"}\n" +
		""
	return tmpl
}
