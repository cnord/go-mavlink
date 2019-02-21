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
		"\tlisteners\tmap[chan []byte](chan bool)\n" +
		"}\n" +
		"\n" +
		"func NewMulticast() Multicast {\n" +
		"\tm := Multicast{\n" +
		"\t\tlisteners: make(map[chan []byte](chan bool)),\n" +
		"\t}\n" +
		"\treturn m\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) register(done chan bool) chan []byte {\n" +
		"\tdata := make(chan []byte)\n" +
		"\tm.Lock()\n" +
		"\tm.listeners[data] = done\n" +
		"\tm.Unlock()\n" +
		"\treturn data\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) notify(buffer []byte) {\n" +
		"\tm.Lock()\n" +
		"\tfor k, _ := range m.listeners {\n" +
		"\t\tk <- buffer\n" +
		"\t}\n" +
		"\tm.Unlock()\n" +
		"}\n" +
		"\n" +
		"func (m *Multicast) clear(data chan []byte) {\n" +
		"\tm.Lock()\n" +
		"\tclose(m.listeners[data])\n" +
		"\tclose(data)\n" +
		"\tdelete(m.listeners, data)\n" +
		"\tm.Unlock()\n" +
		"}\n" +
		"\n" +
		""
	return tmpl
}
