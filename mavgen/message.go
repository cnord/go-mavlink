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
		"{{-if .Mavlink1 }}\n" +
		"import \"fmt\"\n" +
		"{{- end }}\n" +
		"\n" +
		"// zeroTail is a cache of zero slice for auto append tail to\n" +
		"// payload in Mavlink2 messages with trimmed payload (variable length)\n" +
		"var (\n" +
		"\tzeroTail = make([]byte, 256)\n" +
		"{{-if .Mavlink1 }}\n" +
		"    errPayloadTooSmall = fmt.Errorf(\"payload too small\")\n" +
		"{{- end }}\n" +
		")\n" +
		"\n" +
		"// Message is a basic type for encoding/decoding mavlink messages.\n" +
		"// use the Pack() and Unpack() routines on specific message\n" +
		"// types to convert them to/from the Packet type.\n" +
		"type Message interface {\n" +
		"\tDialect() *Dialect\n" +
		"\tPack(*Packet) error\n" +
		"\tUnpack(*Packet) error\n" +
		"\tMsgID() MessageID\n" +
		"\tMsgName() string\n" +
		"}\n" +
		""
	return tmpl
}
