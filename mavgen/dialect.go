/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// dialectTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func dialectTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"// Dialect represents a set of message definitions.\n" +
		"// Some dialects have conflicting definitions for given message IDs,\n" +
		"// so a list of dialects must be provided to an Encoder/Decoder in\n" +
		"// order to specify which packets to use for the conflicting IDs.\n" +
		"//\n" +
		"type Dialect struct {\n" +
		"\tName                      string\n" +
		"\tcrcExtras                 map[MessageID]uint8\n" +
		"\tmessageConstructorByMsgID map[MessageID]func(*Packet) Message\n" +
		"}\n" +
		"\n" +
		"// GetMessage function produce message by packet\n" +
		"func (d *Dialect) GetMessage(pkt *Packet) (msg Message, ok bool) {\n" +
		"\tconstructor, ok := d.messageConstructorByMsgID[pkt.MsgID]\n" +
		"\tif !ok {\n" +
		"\t\treturn nil, false\n" +
		"\t}\n" +
		"\treturn constructor(pkt), true\n" +
		"}\n" +
		"\n" +
		"// DialectSlice typedef is an alias for a slice of Dialect pointers\n" +
		"// Only really intended to be accessed as a field on Encoder/Decoder\n" +
		"type DialectSlice []*Dialect\n" +
		"\n" +
		"// look up the crcextra for msgid\n" +
		"func (ds *DialectSlice) findCrcX(msgid MessageID) (uint8, error) {\n" +
		"\n" +
		"\t// http://www.mavlink.org/mavlink/crc_extra_calculation\n" +
		"\tfor _, d := range *ds {\n" +
		"\t\tif crcx, ok := d.crcExtras[msgid]; ok {\n" +
		"\t\t\treturn crcx, nil\n" +
		"\t\t}\n" +
		"\t}\n" +
		"\n" +
		"\treturn 0, ErrUnknownMsgID\n" +
		"}\n" +
		"\n" +
		"// IndexOf returns the index of d or -1 if not found\n" +
		"func (ds *DialectSlice) IndexOf(d *Dialect) int {\n" +
		"\tfor i, dlct := range *ds {\n" +
		"\t\tif d.Name == dlct.Name {\n" +
		"\t\t\treturn i\n" +
		"\t\t}\n" +
		"\t}\n" +
		"\treturn -1\n" +
		"}\n" +
		"\n" +
		"// Add appends d if not already present in ds\n" +
		"func (ds *DialectSlice) Add(d *Dialect) {\n" +
		"\tif ds.IndexOf(d) < 0 {\n" +
		"\t\t*ds = append(*ds, d)\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"// Remove removes d if present in ds\n" +
		"func (ds *DialectSlice) Remove(d *Dialect) {\n" +
		"\tif i := ds.IndexOf(d); i >= 0 {\n" +
		"\t\t// https://github.com/golang/go/wiki/SliceTricks\n" +
		"\t\t(*ds)[len(*ds)-1], *ds = nil, append((*ds)[:i], (*ds)[i+1:]...)\n" +
		"\t}\n" +
		"}\n" +
		""
	return tmpl
}
