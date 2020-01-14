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
		"// AddDialect append dialect to internal store\n" +
		"func AddDialect(d *Dialect) {\n" +
		"\tdialects.Add(d)\n" +
		"}\n" +
		"\n" +
		"// RemoveDialect remove dialect from internal store\n" +
		"func RemoveDialect(d *Dialect) {\n" +
		"\tdialects.Remove(d)\n" +
		"}\n" +
		"\n" +
		""
	return tmpl
}
