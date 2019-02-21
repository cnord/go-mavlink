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
		"\tMulticast Multicast\n" +
		"\tdata      chan []byte\n" +
		"\tdone      chan bool\n" +
		"\tdecoded\t  chan *Packet\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) PushData(data []byte) {\n" +
		"\td.data <- data\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) NextPacket(duration time.Duration) *Packet {\n" +
		"\tselect {\n" +
		"\tcase packet := <- d.decoded :\n" +
		"\t\treturn packet\n" +
		"\tcase <-time.After(duration) :\n" +
		"\t\treturn nil\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) Stop() {\n" +
		"\td.Multicast.Lock()\n" +
		"\tfor _, v := range d.Multicast.listeners {\n" +
		"\t\tv <- true\n" +
		"\t}\n" +
		"\td.Multicast.Unlock()\n" +
		"}\n" +
		"\n" +
		"// NewChannelDecoder function create decoder instance with default dialect\n" +
		"func NewChannelDecoder() *Decoder {\n" +
		"\td := &Decoder{\n" +
		"\t\tMulticast: NewMulticast(),\n" +
		"\t\tdata : make(chan []byte, 1024),\n" +
		"\t\tdone : make(chan bool),\n" +
		"\t\tdecoded:   make(chan *Packet),\n" +
		"\t}\n" +
		"\tgo func() {\n" +
		"\t\tdefer func() {\n" +
		"\t\t\tclose(d.done)\n" +
		"\t\t\tclose(d.data)\n" +
		"\t\t}()\n" +
		"\t\tfor {\n" +
		"\t\t\tselect {\n" +
		"\t\t\tcase buffer := <-d.data:\n" +
		"\t\t\t\td.Multicast.notify(buffer)\n" +
		"\t\t\t\tfor i, c := range buffer {\n" +
		"\t\t\t\t\tif c == magicNumber {\n" +
		"\t\t\t\t\t\tdone := make(chan bool)\n" +
		"\t\t\t\t\t\tnewBytes := d.Multicast.register(done)\n" +
		"\t\t\t\t\t\tgo func() {\n" +
		"\t\t\t\t\t\t\tdefer d.Multicast.clear(newBytes)\n" +
		"\t\t\t\t\t\t\tvar parser Parser\n" +
		"\t\t\t\t\t\t\tfor {\n" +
		"\t\t\t\t\t\t\t\tselect {\n" +
		"\t\t\t\t\t\t\t\tcase buffer := <-newBytes:\n" +
		"\t\t\t\t\t\t\t\t\tfor _, c := range buffer {\n" +
		"\t\t\t\t\t\t\t\t\t\tpacket, err := parser.parseChar(c)\n" +
		"\t\t\t\t\t\t\t\t\t\tif err != nil {\n" +
		"\t\t\t\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t\t\t\t} else if packet != nil {\n" +
		"\t\t\t\t\t\t\t\t\t\t\td.decoded <- packet\n" +
		"\t\t\t\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t\t\tcase <-done:\n" +
		"\t\t\t\t\t\t\t\t\treturn\n" +
		"\t\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\t}()\n" +
		"\t\t\t\t\t\tnewBytes <- buffer[i:]\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t}\n" +
		"\t\t\tcase <-d.done:\n" +
		"\t\t\t\td.Stop()\n" +
		"\t\t\t\treturn\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\t}()\n" +
		"\treturn d\n" +
		"}\n" +
		""
	return tmpl
}
