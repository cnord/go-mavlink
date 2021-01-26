/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// constantsTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func constantsTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import (\n" +
		"\t\"errors\"\n" +
		")\n" +
		"\n" +
		"// magicNumber constant for mavlink {{if .Mavlink2 -}} 2 {{- else -}} 1 {{- end}}.0\n" +
		"const (\n" +
		"\tmagicNumber = {{if .Mavlink2 -}} 0xfd {{- else -}} 0xfe {{- end}}\n" +
		")\n" +
		"\n" +
		"// MessageID typedef\n" +
		"type MessageID {{if .Mavlink2 -}} uint32 {{else}} uint8 {{- end}}\n" +
		"\n" +
		"\n" +
		"// MAVLINK_PARSE_STATE typedef\n" +
		"type MAVLINK_PARSE_STATE int\n" +
		"\n" +
		"// MAVLINK_PARSE_STATES\n" +
		"const (\n" +
		"\tMAVLINK_PARSE_STATE_UNINIT             MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_IDLE               MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_STX            MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_LENGTH         MAVLINK_PARSE_STATE = iota\n" +
		"{{- if .Mavlink2}}\n" +
		"\tMAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS   MAVLINK_PARSE_STATE = iota\n" +
		"{{- end}}\n" +
		"\tMAVLINK_PARSE_STATE_GOT_SEQ            MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_SYSID          MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_COMPID         MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_MSGID1         MAVLINK_PARSE_STATE = iota\n" +
		"{{- if .Mavlink2}}\n" +
		"\tMAVLINK_PARSE_STATE_GOT_MSGID2         MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_MSGID3         MAVLINK_PARSE_STATE = iota\n" +
		"{{- end}}\n" +
		"\tMAVLINK_PARSE_STATE_GOT_PAYLOAD        MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_CRC1           MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_BAD_CRC        MAVLINK_PARSE_STATE = iota\n" +
		"\tMAVLINK_PARSE_STATE_GOT_GOOD_MESSAGE   MAVLINK_PARSE_STATE = iota\n" +
		")\n" +
		"\n" +
		"var (\n" +
		"\t// ErrUnknownMsgID define\n" +
		"\tErrUnknownMsgID = errors.New(\"unknown msg id\")\n" +
		"\t// ErrCrcFail define\n" +
		"\tErrCrcFail = errors.New(\"checksum did not match\")\n" +
		"\t// ErrNoNewData define\n" +
		"\tErrNoNewData = errors.New(\"No new data\")\n" +
		"\t// ErrNilPointerReference define\n" +
		"\tErrNilPointerReference = errors.New(\"Nil pointer reference\")\n" +
		"\t// currentSeqNum\n" +
		"\tcurrentSeqNum uint8\n" +
		"\t// dialects\n" +
		"\tdialects DialectSlice = DialectSlice{ {{- .DialectName -}} }\n" +
		")\n" +
		""
	return tmpl
}
