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
		"\t\"sort\"\n" +
		"\t\"time\"\n" +
		")\n" +
		"\n" +
		"// Decoder struct provide decoding processor\n" +
		"type Decoder struct {\n" +
		"\tdata    chan []byte\n" +
		"\tdecoded chan *Packet\n" +
		"}\n" +
		"\n" +
		"func (d *Decoder) PushData(data []byte) {\n" +
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
		"\t\tvar parsers []*Parser\n" +
		"\t\tuniqueIndexesToDelete := map[int]*Parser{}\n" +
		"\t\tdefer func() {\n" +
		"\t\t\tfor index := range uniqueIndexesToDelete {\n" +
		"\t\t\t\tparsers[index].Reset()\n" +
		"\t\t\t\tparsersPool.Put(parsers[index])\n" +
		"\t\t\t}\n" +
		"\t\t}()\n" +
		"\t\tvar indexesToDelete []int\n" +
		"\t\tfor {\n" +
		"\t\t\tbuffer, ok := <-d.data\n" +
		"\t\t\tif !ok {\n" +
		"\t\t\t\tclose(d.decoded)\n" +
		"\t\t\t\treturn\n" +
		"\t\t\t}\n" +
		"\t\t\tfor _, c := range buffer {\n" +
		"\t\t\t\tif c == magicNumber {\n" +
		"\t\t\t\t\tparsers = append(parsers, parsersPool.Get().(*Parser))\n" +
		"\t\t\t\t}\n" +
		"\n" +
		"\t\t\t\tfor i, parser := range parsers {\n" +
		"\t\t\t\t\tpacket, err := parser.parseChar(c)\n" +
		"\t\t\t\t\tif err != nil {\n" +
		"\t\t\t\t\t\tuniqueIndexesToDelete[i] = parser\n" +
		"\t\t\t\t\t\tcontinue\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t\tif packet != nil {\n" +
		"\t\t\t\t\t\td.decoded <- packet\n" +
		"\t\t\t\t\t\tfor j := i; j >= 0; j-- {\n" +
		"\t\t\t\t\t\t\tuniqueIndexesToDelete[i] = parser\n" +
		"\t\t\t\t\t\t}\n" +
		"\t\t\t\t\t\tcontinue\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t}\n" +
		"\n" +
		"\t\t\t\tif len(uniqueIndexesToDelete) != 0 {\n" +
		"\t\t\t\t\tfor index := range uniqueIndexesToDelete {\n" +
		"\t\t\t\t\t\tindexesToDelete = append(indexesToDelete, index)\n" +
		"\t\t\t\t\t\tdelete(uniqueIndexesToDelete, index)\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t\tif len(indexesToDelete) > 1 {\n" +
		"\t\t\t\t\t\tsort.Ints(indexesToDelete)\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t\tfor i := len(indexesToDelete) - 1; i >= 0; i-- {\n" +
		"\t\t\t\t\t\tindex := indexesToDelete[i]\n" +
		"\t\t\t\t\t\tparsers[index].Reset()\n" +
		"\t\t\t\t\t\tparsersPool.Put(parsers[index])\n" +
		"\t\t\t\t\t\tcopy(parsers[index:], parsers[index+1:])\n" +
		"\t\t\t\t\t\tparsers[len(parsers)-1] = nil\n" +
		"\t\t\t\t\t\tparsers = parsers[:len(parsers)-1]\n" +
		"\t\t\t\t\t}\n" +
		"\t\t\t\t\tindexesToDelete = indexesToDelete[:0]\n" +
		"\t\t\t\t}\n" +
		"\t\t\t}\n" +
		"\t\t}\n" +
		"\t}()\n" +
		"\treturn d\n" +
		"}\n" +
		""
	return tmpl
}
