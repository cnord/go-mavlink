package main

import (
	"bytes"
	"encoding/xml"
	"errors"
	"fmt"
	"github.com/howeyc/crc16"
	"go/format"
	"io"
	"io/ioutil"
	"sort"
	"strconv"
	"strings"
	"text/template"
)

var (
	upperCaseWords = map[string]string{
		"Id":   "ID",
		"Cpu":  "CPU",
		"Uri":  "URI",
		"Url":  "URL",
		"Uid":  "UID",
		"Http": "HTTP",
		"Udp":  "UDP",
	}
)

// Dialect described root tag of schema
type Dialect struct {
	Name           string
	MavlinkVersion int

	XMLName  xml.Name   `xml:"mavlink"`
	Version  string     `xml:"version"`
	Include  string     `xml:"include"`
	Enums    []*Enum    `xml:"enums>enum"`
	Messages []*Message `xml:"messages>message"`
}

// Enum described schema tag enum
type Enum struct {
	Name        string       `xml:"name,attr"`
	Description string       `xml:"description"`
	Entries     []*EnumEntry `xml:"entry"`
}

// EnumEntry described schema tag entry
type EnumEntry struct {
	Value       uint32            `xml:"value,attr"`
	Name        string            `xml:"name,attr"`
	Description string            `xml:"description"`
	Params      []*EnumEntryParam `xml:"param"`
}

// EnumEntryParam described schema tag param
type EnumEntryParam struct {
	Index       uint8  `xml:"index,attr"`
	Description string `xml:",innerxml"`
}

// Message described schema tag message
type Message struct {
	// use uint32 instead of uint8 so that we can filter
	// msgids from mavlink v2, which are 24 bits wide.
	// see filtering in ParseDialect()
	ID          uint32          `xml:"id,attr"`
	Name        string          `xml:"name,attr"`
	Description string          `xml:"description"`
	Fields      []*MessageField `xml:"field"`

	// this field is only used during ParseDialect phase,
	// it contains an empty string after ParseDialect returns
	Raw string `xml:",innerxml"`
}

// MessageField described schema tag filed
type MessageField struct {
	CType       string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Enum        string `xml:"enum,attr"`
	Description string `xml:",innerxml"`
	GoType      string
	BitSize     int
	ArrayLen    int
	ByteOffset  int // from beginning of payload
}

var funcMap = template.FuncMap{
	"UpperCamelCase": UpperCamelCase,
}

// SizeInBytes function calculate size in bytes of message field
func (f *MessageField) SizeInBytes() int {
	if f.ArrayLen > 0 {
		return f.BitSize / 8 * f.ArrayLen
	}
	return f.BitSize / 8
}

// Size function calculate size in bytes of message
func (m *Message) Size() int {
	sz := 0
	for _, f := range m.Fields {
		sz += f.SizeInBytes()
	}
	return sz
}

// CRCExtra calculation: http://www.mavlink.org/mavlink/crc_extra_calculation
func (m *Message) CRCExtra() uint8 {
	hash := crc16.New(crc16.CCITTTable)

	fmt.Fprint(hash, m.Name+" ")
	for _, f := range m.Fields {
		cType := f.CType
		if cType == "uint8_t_mavlink_version" {
			cType = "uint8_t"
		}
		// type name for crc extra purposes does not include array portion
		if idx := strings.IndexByte(cType, '['); idx >= 0 {
			cType = cType[:idx]
		}
		fmt.Fprint(hash, cType+" "+f.Name+" ")
		if f.ArrayLen > 0 {
			_, _ = hash.Write([]byte{byte(f.ArrayLen)})
		}
	}

	crc := hash.Sum16()
	return uint8((crc & 0xFF) ^ (crc >> 8))
}

// implementation of sort.Interface for Message
func (m *Message) Len() int {
	return len(m.Fields)
}

func (m *Message) Less(i, j int) bool {
	return m.Fields[i].BitSize < m.Fields[j].BitSize
}

func (m *Message) Swap(i, j int) {
	m.Fields[i], m.Fields[j] = m.Fields[j], m.Fields[i]
}

// UpperCamelCase function convert names to upper camel case
func UpperCamelCase(s string) string {
	var b bytes.Buffer
	for _, fragment := range strings.Split(s, "_") {
		if len(fragment) > 0 {
			word := strings.ToUpper(fragment[:1]) + strings.ToLower(fragment[1:])
			if replacement, ok := upperCaseWords[word]; ok {
				word = replacement
			}
			b.WriteString(word)
		}
	}
	return b.String()
}

// helper to pack a single element into a payload.
// can be called for a single field, or an element within a field's array.
func (f *MessageField) payloadPackPrimitive(offset, name string) string {

	if f.BitSize == 8 {
		return fmt.Sprintf("payload[%s] = byte(%s)", offset, name)
	}

	if f.IsFloat() {
		switch f.BitSize {
		case 32, 64:
			return fmt.Sprintf("binary.LittleEndian.PutUint%d(payload[%s:], math.Float%dbits(%s))", f.BitSize, offset, f.BitSize, name)
		}
	} else {
		switch f.BitSize {
		case 16, 32, 64:
			return fmt.Sprintf("binary.LittleEndian.PutUint%d(payload[%s:], uint%d(%s))", f.BitSize, offset, f.BitSize, name)
		}
	}

	panic("unhandled bitsize")
}

// PayloadPackSequence function produce a string that will pack
// this message's fields into a byte slice called 'payload'
func (f *MessageField) PayloadPackSequence() string {
	name := UpperCamelCase(f.Name)

	if f.ArrayLen > 0 {
		// optimize to copy() if possible
		if strings.HasSuffix(f.GoType, "byte") || strings.HasSuffix(f.GoType, "uint8") {
			return fmt.Sprintf("copy(payload[%d:], m.%s[:])", f.ByteOffset, name)
		}

		// pack each element in the array
		s := fmt.Sprintf("for i, v := range m.%s {\n", name)
		off := fmt.Sprintf("%d + i * %d", f.ByteOffset, f.BitSize/8)
		s += f.payloadPackPrimitive(off, "v") + "\n"
		s += fmt.Sprintf("}")
		return s
	}

	// pack a single field
	return f.payloadPackPrimitive(fmt.Sprintf("%d", f.ByteOffset), "m."+name)
}

func (f *MessageField) payloadUnpackPrimitive(offset string) string {

	if f.BitSize == 8 {
		return fmt.Sprintf("%s(payload[%s])", goArrayType(f.GoType), offset)
	}

	if f.IsFloat() {
		switch f.BitSize {
		case 32, 64:
			return fmt.Sprintf("math.Float%dfrombits(binary.LittleEndian.Uint%d(payload[%s:]))", f.BitSize, f.BitSize, offset)
		}
	} else {
		switch f.BitSize {
		case 16, 32, 64:
			return fmt.Sprintf("%s(binary.LittleEndian.Uint%d(payload[%s:]))", goArrayType(f.GoType), f.BitSize, offset)
		}
	}

	panic("unhandled bitsize")
}

// PayloadUnpackSequence function produce a string that will unpack
// this message's fields into a byte slice called 'payload'
func (f *MessageField) PayloadUnpackSequence() string {
	name := UpperCamelCase(f.Name)

	if f.ArrayLen > 0 {
		// optimize to copy() if possible
		if strings.HasSuffix(f.GoType, "byte") || strings.HasSuffix(f.GoType, "uint8") {
			return fmt.Sprintf("copy(m.%s[:], payload[%d:%d])", name, f.ByteOffset, f.ByteOffset+f.ArrayLen)
		}

		// unpack each element in the array
		s := fmt.Sprintf("for i := 0; i < len(m.%s); i++ {\n", name)
		off := fmt.Sprintf("%d + i * %d", f.ByteOffset, f.BitSize/8)
		s += fmt.Sprintf("m.%s[i] = %s\n", name, f.payloadUnpackPrimitive(off))
		s += fmt.Sprintf("}")
		return s
	}

	return fmt.Sprintf("m.%s = %s", name, f.payloadUnpackPrimitive(fmt.Sprintf("%d", f.ByteOffset)))
}

// func SanitizeComments(s string) string {
// 	return strings.Replace(s, "\n", "\n// ", -1)
// }

// Return the number of non-extension fields, that are contained in rawmsg, where rawmsg
// is raw XML content of "message" element.
func numBaseFields(rawmsg string) int {
	dec := xml.NewDecoder(strings.NewReader(rawmsg))

	fields := 0
	for {
		t, err := dec.Token()
		if err != nil {
			if err != io.EOF {
				panic(err)
			}
			return fields
		}
		if se, ok := t.(xml.StartElement); ok {
			switch se.Name.Local {
			case "field":
				fields++
			case "extensions":
				return fields
			}
			if err := dec.Skip(); err != nil {
				panic(err)
			}
		}
	}
}

// ParseDialect read in an xml-based dialect file,
// and populate a Dialect struct with its contents
func ParseDialect(in io.Reader, name string) (*Dialect, error) {

	filebytes, err := ioutil.ReadAll(in)
	if err != nil {
		return nil, err
	}

	dialect := &Dialect{
		Name: name,
	}

	if err := xml.Unmarshal(filebytes, &dialect); err != nil {
		return nil, err
	}

	// filter out messages with MSG_ID > 256. these are from
	// mavlink v2 and do not fit in uint8
	filteredMessages := make([]*Message, len(dialect.Messages))
	n := 0
	for _, msg := range dialect.Messages {
		if msg.ID <= 0xff {
			// we ignore field extensions, since they are from
			// mavlink v2
			ind := numBaseFields(msg.Raw)
			if ind >= 0 {
				msg.Fields = msg.Fields[:ind]
			}
			filteredMessages[n] = msg
			n++
		}
		msg.Raw = ""
	}
	dialect.Messages = filteredMessages[:n]

	// if dialect.Include != "" {
	// 	generate(protocol.IncludeName())
	// }

	return dialect, nil
}

// convert a C primitive type to its corresponding Go type.
// do not handle arrays or other constructs...just primitives.
func c2goPrimitive(ctype string) string {
	switch ctype {
	case "uint8_t", "uint16_t", "uint32_t", "uint64_t",
		"int8_t", "int16_t", "int32_t", "int64_t":
		idx := strings.IndexByte(ctype, '_')
		return ctype[:idx]
	case "char":
		return "byte"
	case "float":
		return "float32"
	case "double":
		return "float64"
	case "uint8_t_mavlink_version":
		return "uint8"
	default:
		panic(fmt.Sprintf("c2goPrimitive: unhandled primitive type - %s", ctype))
	}
}

func goArrayType(s string) string {
	idx := strings.IndexByte(s, ']')
	if idx < 0 {
		return s
	}
	return s[idx+1:]
}

// IsFloat return check state of validating field as float type
func (f *MessageField) IsFloat() bool {
	return strings.HasPrefix(goArrayType(f.GoType), "float")
}

// GoTypeInfo produce type info string
func GoTypeInfo(s string) (string, int, int, error) {

	var name string
	var bitsz, arraylen int
	var err error

	// array? leave the [N] but convert the primitive type name
	if idx := strings.IndexByte(s, '['); idx < 0 {
		name = c2goPrimitive(s)
	} else {
		name = s[idx:] + c2goPrimitive(s[:idx])
		if arraylen, err = strconv.Atoi(s[idx+1 : len(s)-1]); err != nil {
			return "", 0, 0, err
		}
	}

	// determine bit size for this type
	if strings.HasSuffix(name, "byte") {
		bitsz = 8
	} else {
		t := name[strings.IndexByte(name, ']')+1:]
		if sizeStart := strings.IndexAny(t, "8136"); sizeStart != -1 {
			if bitsz, err = strconv.Atoi(t[sizeStart:]); err != nil {
				return "", 0, 0, err
			}
		} else {
			return "", 0, 0, errors.New("Unknown message field size")
		}
	}

	return name, bitsz, arraylen, nil
}

// GenerateGo generate a .go source file from the given dialect
func (d *Dialect) GenerateGo(w io.Writer) error {
	// templatize to buffer, format it, then write out

	var bb bytes.Buffer

	bb.WriteString("//////////////////////////////////////////////////\n")
	bb.WriteString("//\n")
	bb.WriteString("// NOTE: do not edit,\n")
	bb.WriteString("// this file created automatically by mavgen.go\n")
	bb.WriteString("//\n")
	bb.WriteString("//////////////////////////////////////////////////\n\n")

	bb.WriteString("package mavlink\n\n")

	bb.WriteString("import (\n")
	bb.WriteString("\"encoding/binary\"\n")
	bb.WriteString("\"math\"\n")
	bb.WriteString(")\n")

	err := d.generateEnums(&bb)
	if err != nil {
		return err
	}
	err = d.generateClasses(&bb)
	if err != nil {
		return err
	}
	err = d.generateMsgIds(&bb)
	if err != nil {
		return err
	}

	formatted, err := format.Source(bb.Bytes())
	if err != nil {
		formatted = bb.Bytes()
	}

	n, err := w.Write(formatted)
	if err == nil && n != len(formatted) {
		return io.ErrShortWrite
	}

	return err
}

func (d *Dialect) generateEnums(w io.Writer) error {
	enumTmpl := `
{{range .Enums}}
// {{.Name}} (generated enum)
// {{.Description}}
const ({{range .Entries}}
	{{.Name}} = {{.Value}} // {{.Description}}{{end}}
)
{{end}}
`
	// fill in missing enum values if necessary, and ensure description strings are valid.
	for _, e := range d.Enums {
		e.Description = strings.Replace(e.Description, "\n", " ", -1)
		e.Name = UpperCamelCase(e.Name)
		for i, ee := range e.Entries {
			if ee.Value == 0 {
				ee.Value = uint32(i)
			}
			ee.Description = strings.Trim(strings.Replace(ee.Description, "\n", " ", -1), " .")
			if len(ee.Params) > 0 {
				if len(ee.Description) > 0 {
					ee.Description += ". "
				}
				ee.Description += "Params: "
			}
			for _, pp := range ee.Params {
				ee.Description += strconv.Itoa(int(pp.Index)) + ") " + pp.Description + "; "
			}
		}
	}

	return template.Must(template.New("enums").Parse(enumTmpl)).Execute(w, d)
}

func (d *Dialect) generateMsgIds(w io.Writer) error {
	msgIDTmpl := `
{{$dialect := .Name | UpperCamelCase}}
// Message IDs
const ({{range .Messages}}
	MSG_ID_{{.Name}} MessageID = {{.ID}}{{end}}
)

// Dialect{{.Name | UpperCamelCase}} is the dialect represented by {{.Name}}.xml
var Dialect{{.Name | UpperCamelCase}} = &Dialect{
	Name: "{{.Name}}",
	crcExtras: map[MessageID]uint8{ {{range .Messages}}
		MSG_ID_{{.Name}}: {{.CRCExtra}},{{end}}
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{ {{range .Messages}}
		MSG_ID_{{.Name}}: func(pkt *Packet) Message {
			msg := new({{$dialect}}{{.Name | UpperCamelCase}})
			msg.Unpack(pkt)
			return msg
		},{{end}}
	},
}
`
	return template.Must(template.New("msgIds").Funcs(funcMap).Parse(msgIDTmpl)).Execute(w, d)
}

// generate class definitions for each msg id.
// for now, pack/unpack payloads via encoding/binary since it
// is expedient and correct. optimize this if/when needed.
func (d *Dialect) generateClasses(w io.Writer) error {

	classesTmpl := `
{{$mavlinkVersion := .MavlinkVersion}}
{{$dialect := .Name | UpperCamelCase}}
{{range .Messages}}
{{$name := .Name | UpperCamelCase}}
// {{$dialect}}{{$name}} struct (generated typeinfo)  
// {{.Description}}
type {{$dialect}}{{$name}} struct { {{range .Fields}}
  {{.Name | UpperCamelCase}} {{.GoType}} // {{.Description}}{{end}}
}

// Dialect (generated function)
func (m *{{$dialect}}{{$name}}) Dialect() *Dialect {
	return Dialect{{$dialect}}
}

// MsgID (generated function)
func (m *{{$dialect}}{{$name}}) MsgID() MessageID {
	return MSG_ID_{{.Name}}
}

// MsgName (generated function)
func (m *{{$dialect}}{{$name}}) MsgName() string {
	return "{{.Name | UpperCamelCase}}"
}

// Pack (generated function)
func (m *{{$dialect}}{{$name}}) Pack(p *Packet) error {
	payload := make([]byte, {{ .Size }}){{range .Fields}}
	{{.PayloadPackSequence}}{{end}}
{{- if gt $mavlinkVersion 1 }}
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
{{- end }}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *{{$dialect}}{{$name}}) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < {{ .Size }} {
{{- if eq $mavlinkVersion 1 }}
		return errPayloadTooSmall
{{- else }}
		payload = append(payload, zeroTail[:{{ .Size }}-len(p.Payload)]...)
{{- end }}
	}{{range .Fields}}
	{{.PayloadUnpackSequence}}{{end}}
	return nil
}
{{end}}
`
	for _, m := range d.Messages {
		m.Description = strings.Replace(m.Description, "\n", "\n// ", -1)

		for _, f := range m.Fields {
			f.Description = strings.Replace(f.Description, "\n", " ", -1)
			goname, gosz, golen, err := GoTypeInfo(f.CType)
			if err != nil {
				return err
			}
			f.GoType, f.BitSize, f.ArrayLen = goname, gosz, golen
		}

		// ensure fields are sorted according to their size,
		// http://www.mavlink.org/mavlink/crc_extra_calculation
		sort.Stable(sort.Reverse(m))

		// once sorted, calculate offsets for use in payload packing/unpacking
		offset := 0
		for _, f := range m.Fields {
			f.ByteOffset = offset
			offset += f.SizeInBytes()
		}
	}

	return template.Must(template.New("classesTmpl").Funcs(funcMap).Parse(classesTmpl)).Execute(w, d)
}
