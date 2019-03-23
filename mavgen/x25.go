/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// x25Template is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func x25Template() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"// X25 implements hash.Hash\n" +
		"type X25 struct {\n" +
		"\tcrc uint16\n" +
		"}\n" +
		"\n" +
		"// New function create crc calculator\n" +
		"func NewX25() *X25 {\n" +
		"\tx := &X25{}\n" +
		"\tx.Reset()\n" +
		"\treturn x\n" +
		"}\n" +
		"\n" +
		"// WriteByte function append byte into crc accumulation\n" +
		"func (x *X25) WriteByte(b byte) {\n" +
		"\ttmp := b ^ byte(x.crc&0xff)\n" +
		"\ttmp ^= (tmp << 4)\n" +
		"\tx.crc = (x.crc >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)\n" +
		"\treturn nil\n" +
		"}\n" +
		"\n" +
		"// Write function append byte slice into crc accumulation\n" +
		"func (x *X25) Write(p []byte) (n int, err error) {\n" +
		"\tfor i, b := range p {\n" +
		"\t\tx.WriteByte(b)\n" +
		"\t}\n" +
		"\treturn len(p), nil\n" +
		"}\n" +
		"\n" +
		"// Sum16 function return accumulated crc\n" +
		"func (x *X25) Sum16() uint16 { return x.crc }\n" +
		"\n" +
		"// Sum function append crc bytes on byte slice\n" +
		"func (x *X25) Sum(in []byte) []byte {\n" +
		"\ts := x.Sum16()\n" +
		"\treturn append(in, byte(s>>8), byte(s))\n" +
		"}\n" +
		"\n" +
		"// Size function return crc size in bytes\n" +
		"func (x *X25) Size() int { return 2 }\n" +
		"\n" +
		"// BlockSize function return crc size in blocks\n" +
		"func (x *X25) BlockSize() int { return 1 }\n" +
		"\n" +
		"// Reset function reset current crc\n" +
		"func (x *X25) Reset() { x.crc = 0xffff }\n" +
		""
	return tmpl
}
