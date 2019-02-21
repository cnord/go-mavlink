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
		"// Message is a basic type for encoding/decoding mavlink messages.\n" +
		"// use the Pack() and Unpack() routines on specific message\n" +
		"// types to convert them to/from the Packet type.\n" +
		"type Message interface {\n" +
		"\tPack(*Packet) error\n" +
		"\tUnpack(*Packet) error\n" +
		"\tMsgID() MessageID\n" +
		"\tMsgName() string\n" +
		"}\n" +
		""
	return tmpl
}
