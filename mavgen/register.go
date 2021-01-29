/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// registerTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func registerTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import \"strconv\"\n" +
		"\n" +
		"// Register method provide register dialect message on decoder knowledge\n" +
		"func Register(msgID MessageID, msgName string, crcExtra uint8, msgConstructor func(p *Packet) Message) {\n" +
		"\tif exists, ok := msgNames[msgID]; ok {\n" +
		"\t\tpanic(\"Message with ID = \" + strconv.Itoa(int(msgID)) + \" already exists. Fix collision '\" + msgName + \"' vs '\" + exists + \"' and re-run mavgen\")\n" +
		"\t} else {\n" +
		"\t\tmsgNames[msgID] = \"MSG_ID_SENSOR_OFFSETS\"\n" +
		"\t\tmsgCrcExtras[msgID] = crcExtra\n" +
		"\t\tmsgConstructors[msgID] = msgConstructor\n" +
		"\t}\n" +
		"}\n" +
		""
	return tmpl
}
