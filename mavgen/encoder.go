/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// encoderTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func encoderTemplate() string {
	var tmpl = "/*\n" +
		" * CODE GENERATED AUTOMATICALLY WITH\n" +
		" *    github.com/asmyasnikov/go-mavlink/mavgen\n" +
		" * THIS FILE SHOULD NOT BE EDITED BY HAND\n" +
		" */\n" +
		"\n" +
		"package mavlink\n" +
		"\n" +
		"import (\n" +
		"\t\"fmt\"\n" +
		"\t\"io\"\n" +
		")\n" +
		"\n" +
		"// Decoder struct provide decoding processor\n" +
		"type Encoder struct {\n" +
		"\twriter  io.Writer\n" +
		"}\n" +
		"\n" +
		"// Decode decode input stream to packet. Method return error or nil\n" +
		"func (e *Encoder) Encode(v interface{}) error {\n" +
		"\tpacket, ok := v.(*Packet)\n" +
		"\tif !ok {\n" +
		"\t\treturn fmt.Errorf(\"cast interface '%+v' to Packet fail\", v)\n" +
		"\t}\n" +
		"\tb := packet.Bytes()\n" +
		"\tn, err := e.writer.Write(b)\n" +
		"\tif len(b) != n {\n" +
		"\t\treturn fmt.Errorf(\"writed %d bytes, but need to write %d bytes\", n, len(b))\n" +
		"\t}\n" +
		"\treturn err\n" +
		"}\n" +
		"\n" +
		"// NewDecoder function create decoder instance with default dialect\n" +
		"func NewEncoder(w io.Writer) *Encoder {\n" +
		"\treturn &Encoder{\n" +
		"\t\twriter:  w,\n" +
		"\t}\n" +
		"}\n" +
		""
	return tmpl
}
