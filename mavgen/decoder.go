/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// decoderTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func decoderTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import (\n" +
		"\t\"bufio\"\n" +
		"\t\"fmt\"\n" +
		"\t\"io\"\n" +
		")\n" +
		"\n" +
		"// Decoder struct provide decoding processor\n" +
		"type Decoder struct {\n" +
		"\treader      io.ByteReader\n" +
		"\tparsers     []*Parser\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) clearParser(parser *Parser) {\n" +
		"\tparser.Reset()\n" +
		"\tparsersPool.Put(parser)\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) clearParsers() {\n" +
		"\tfor _, parser := range d.parsers {\n" +
		"\t\td.clearParser(parser)\n" +
		"\t}\n" +
		"\td.parsers = d.parsers[:0]\n" +
		"}\n" +
		"\n" +
		"// Decode decode input stream to packet. Method return error or nil\n" +
		"func (d *Decoder) Decode(v interface{}) error {\n" +
		"\tpacket, ok := v.(*Packet)\n" +
		"\tif !ok {\n" +
		"\t\treturn fmt.Errorf(\"cast interface '%+v' to Packet fail\", v)\n" +
		"\t}\n" +
		"\tfor {\n" +
		"\t\tc, err := d.reader.ReadByte()\n" +
		"\t\tif err != nil {\n" +
		"\t\t\treturn err\n" +
		"\t\t}\n" +
		"\t\tif c == magicNumber {\n" +
		"\t\t\td.parsers = append(d.parsers, parsersPool.Get().(*Parser))\n" +
		"\t\t}\n" +
		"\t\tparsers := make([]*Parser, 0, len(d.parsers))\n" +
		"\t\tfor _, parser := range d.parsers {\n" +
		"\t\t\tif p, err := parser.parseChar(c); err != nil {\n" +
		"\t\t\t\td.clearParser(parser)\n" +
		"\t\t\t} else if p != nil {\n" +
		"{{- if eq .MavlinkVersion 2}}\n" +
		"\t\t\t\tpacket.InCompatFlags = p.InCompatFlags\n" +
		"\t\t\t\tpacket.CompatFlags = p.CompatFlags\n" +
		"{{- end}}\n" +
		"\t\t\t\tpacket.SeqID = p.SeqID\n" +
		"\t\t\t\tpacket.SysID = p.SysID\n" +
		"\t\t\t\tpacket.CompID = p.CompID\n" +
		"\t\t\t\tpacket.MsgID = p.MsgID\n" +
		"\t\t\t\tpacket.Checksum = p.Checksum\n" +
		"\t\t\t\tpacket.Payload = append(packet.Payload[:0], p.Payload...)\n" +
		"\t\t\t\td.clearParsers()\n" +
		"\t\t\t\treturn nil\n" +
		"\t\t\t} else {\n" +
		"\t\t\t\tparsers = append(parsers, parser)\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\t\td.parsers = parsers\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"func byteReader(r io.Reader) io.ByteReader {\n" +
		"\tif rb, ok := r.(io.ByteReader); ok {\n" +
		"\t\treturn rb\n" +
		"\t}\n" +
		"    return bufio.NewReader(r)\n" +
		"}\n" +
		"\n" +
		"// NewDecoder function create decoder instance\n" +
		"func NewDecoder(r io.Reader) *Decoder {\n" +
		"\treturn &Decoder{\n" +
		"\t\treader:  byteReader(r),\n" +
		"\t\tparsers: make([]*Parser, 0),\n" +
		"\t}\n" +
		"}\n" +
		""
	return tmpl
}
