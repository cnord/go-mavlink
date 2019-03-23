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
		"\t\"time\"\n" +
		")\n" +
		"\n" +
		"// Decoder struct provide decoding processor\n" +
		"type Decoder struct {\n" +
		"\tmulticast Multicast\n" +
		"\tdata      chan []byte\n" +
		"\tdecoded   chan *Packet\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) PushData(data []byte) {\n" +
		"\tdata = append(data[:0:0], data...)\n" +
		"\td.data <- data\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) NextPacket(duration time.Duration) *Packet {\n" +
		"\tselect {\n" +
		"\tcase packet, ok := <-d.decoded:\n" +
		"\t\tif ok {\n" +
		"\t\t\treturn packet\n" +
		"\t\t}\n" +
		"\t\treturn nil\n" +
		"\tcase <-time.After(duration):\n" +
		"\t\treturn nil\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"// Stop make safely stop of decoder\n" +
		"func (d *Decoder) Stop() {\n" +
		"\tclose(d.data)\n" +
		"}\n" +
		"\n" +
		"// NewChannelDecoder function create decoder instance with default dialect\n" +
		"func NewChannelDecoder() *Decoder {\n" +
		"\td := &Decoder{\n" +
		"\t\tdata:    make(chan []byte, 256),\n" +
		"\t\tdecoded: make(chan *Packet, 256),\n" +
		"\t}\n" +
		"\tgo func() {\n" +
		"\t\tfor {\n" +
		"\t\t\tbuffer, ok := <-d.data\n" +
		"\t\t\tif !ok {\n" +
		"\t\t\t\td.multicast.close()\n" +
		"\t\t\t\tclose(d.decoded)\n" +
		"\t\t\t\treturn\n" +
		"\t\t\t}\n" +
		"\t\t\td.multicast.notify(buffer)\n" +
		"\t\t\tfor i, c := range buffer {\n" +
		"\t\t\t\tif c == magicNumber {\n" +
		"\t\t\t\t\tnewBytes := d.multicast.register()\n" +
		"\t\t\t\t\tgo func() {\n" +
		"\t\t\t\t\t\tdefer d.multicast.clear(newBytes)\n" +
		"\t\t\t\t\t\tvar parser Parser\n" +
		"\t\t\t\t\t\tfor {\n" +
		"\t\t\t\t\t\t\tbuffer, ok := <-newBytes\n" +
		"\t\t\t\t\t\t\tif !ok {\n" +
		"\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t\tfor _, c := range buffer {\n" +
		"\t\t\t\t\t\t\t\tpacket, err := parser.parseChar(c)\n" +
		"\t\t\t\t\t\t\t\tif err != nil {\n" +
		"\t\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t\t} else if packet != nil {\n" +
		"\t\t\t\t\t\t\t\t\td.decoded <- packet\n" +
		"\t\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t}()\n" +
		"\t\t\t\t\tnewBytes <- buffer[i:]\n" +
		"\t\t\t\t}\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\t}()\n" +
		"\treturn d\n" +
		"}\n" +
		""
	return tmpl
}
