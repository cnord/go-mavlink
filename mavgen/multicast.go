/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// multicastTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func multicastTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"import (\n" +
		"\t\"sync\"\n" +
		")\n" +
		"\n" +
		"type Multicast struct {\n" +
		"\tsync.Mutex\n" +
		"\tlisteners []chan []byte\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) register() chan []byte {\n" +
		"\tdata := make(chan []byte)\n" +
		"\tm.Lock()\n" +
		"\tdefer m.Unlock()\n" +
		"\tm.listeners = append(m.listeners, data)\n" +
		"\treturn data\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) notify(buffer []byte) {\n" +
		"\tm.Lock()\n" +
		"\tdefer m.Unlock()\n" +
		"\tfor _, data := range m.listeners {\n" +
		"\t\tdata <- buffer\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) clear(data chan []byte) {\n" +
		"\tm.Lock()\n" +
		"\tdefer m.Unlock()\n" +
		"\tfor i, d := range m.listeners {\n" +
		"\t\tif d == data {\n" +
		"\t\t\tm.listeners = append(m.listeners[:i], m.listeners[i+1:]...)\n" +
		"\t\t\tclose(data)\n" +
		"\t\t\treturn\n" +
		"\t\t}\n" +
		"\t}\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) close() {\n" +
		"\tm.Lock()\n" +
		"\tdefer m.Unlock()\n" +
		"\tfor _, d := range m.listeners {\n" +
		"\t\tclose(d)\n" +
		"\t}\n" +
		"\tm.listeners = m.listeners[:0:0]\n" +
		"}\n" +
		""
	return tmpl
}
