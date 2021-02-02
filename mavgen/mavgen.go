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
	"os"
	"path/filepath"
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
	MavlinkVersion int
	FilePath       string
	XMLName        xml.Name   `xml:"mavlink"`
	Version        string     `xml:"version"`
	Include        []string   `xml:"include"`
	Enums          []*Enum    `xml:"enums>enum"`
	Messages       []*Message `xml:"messages>message"`
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
	Raw         string `xml:",innerxml"`
	DialectName string
}

// MessageField described schema tag filed
type MessageField struct {
	CType       string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Enum        string `xml:"enum,attr"`
	Description string `xml:",innerxml"`
	GoType      string
	Tag         string
	BitSize     int
	ArrayLen    int
	ByteOffset  int // from beginning of payload
}

var funcMap = template.FuncMap{
	"UpperCamelCase":   UpperCamelCase,
	"IsByteArrayField": IsByteArrayField,
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

// IsByteArrayField function check field is bytearray
func IsByteArrayField(v interface{}) bool {
	if field, ok := v.(*MessageField); ok {
		if field.ArrayLen == 0 {
			return false
		}
		t := strings.ToLower(field.GoType)
		for _, s := range []string{"byte", "uint8"} {
			if strings.Contains(t, s) {
				return true
			}
		}
	}
	return false
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
		if len(f.Enum) > 0 {
			return fmt.Sprintf("%s(payload[%s])", goArrayType(f.Enum), offset)
		}
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
			if len(f.Enum) > 0 {
				return fmt.Sprintf("%s(binary.LittleEndian.Uint%d(payload[%s:]))", goArrayType(f.Enum), f.BitSize, offset)
			}
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

// parseDialect read in an xml-based dialect stream,
// and populate a Dialect struct with its contents
func parseDialect(in io.Reader) (*Dialect, error) {
	filebytes, err := ioutil.ReadAll(in)
	if err != nil {
		return nil, err
	}

	dialect := &Dialect{}

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

	return dialect, nil
}

// ParseDialect read in an xml-based dialect file,
// and populate a Dialect struct with its contents
func ParseDialect(schemeFile string) (*Dialect, error) {
	in, err := os.Open(schemeFile)
	if err != nil {
		return nil, err
	}
	defer in.Close()
	d, err := parseDialect(in)
	if err != nil {
		return nil, err
	}
	for _, i := range d.Include {
		includedPath := filepath.Join(filepath.Dir(schemeFile), i)
		included, err := ParseDialect(includedPath)
		if err != nil {
			return nil, err
		}
		included.FilePath = includedPath
		if err := d.merge(included); err != nil {
			return nil, err
		}
	}
	return d, nil
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

func (d *Dialect) needImportParentMavlink() bool {
	return len(d.Messages) > 0
}

func (d *Dialect) needImportFmt() bool {
	return len(d.Messages) > 0
}

func (d *Dialect) needImportMath() bool {
	for _, m := range d.Messages {
		for _, f := range m.Fields {
			switch f.CType {
			case "float", "double":
				return true
			}
		}
	}
	return false
}

func (d *Dialect) needImportEncodingBinary() bool {
	for _, m := range d.Messages {
		for _, f := range m.Fields {
			switch f.CType {
			case "uint16_t", "uint32_t", "uint64_t":
				return true
			}
		}
	}
	return false
}

func (d *Dialect) generateGo(w io.Writer, packageName string) error {
	// templatize to buffer, format it, then write out

	var bb bytes.Buffer

	bb.WriteString("//////////////////////////////////////////////////\n")
	bb.WriteString("//\n")
	bb.WriteString("// NOTE: do not edit,\n")
	bb.WriteString("// this file created automatically by mavgen.go\n")
	bb.WriteString("//\n")
	bb.WriteString("//////////////////////////////////////////////////\n\n")

	bb.WriteString("package " + strings.ToLower(packageName) + "\n\n")

	needImportParentMavlink := d.needImportParentMavlink()
	needImportEncodingBinary := d.needImportEncodingBinary()
	needImportFmt := d.needImportFmt()
	needImportMath := d.needImportMath()

	if needImportParentMavlink || needImportEncodingBinary || needImportFmt || needImportMath {
		bb.WriteString("import (\n")
		if needImportParentMavlink {
			bb.WriteString("mavlink \"github.com/asmyasnikov/go-mavlink/generated/mavlink" + strconv.Itoa(d.MavlinkVersion) + "\"\n")
		}
		if needImportEncodingBinary {
			bb.WriteString("\"encoding/binary\"\n")
		}
		if needImportFmt {
			bb.WriteString("\"fmt\"\n")
		}
		if needImportMath {
			bb.WriteString("\"math\"\n")
		}
		bb.WriteString(")\n")
	}

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
{{$enumName := .Name}}
// {{$enumName}} type{{if .Description}}. {{.Description}}{{end}}
type {{.Name}} int

const ({{range .Entries}}
	// {{.Name}} enum{{if .Description}}. {{.Description}}{{end}}
	{{.Name}} {{$enumName}} = {{.Value}} {{end}}
)
{{end}}
`
	// fill in missing enum values if necessary, and ensure description strings are valid.
	for _, e := range d.Enums {
		e.Description = strings.Replace(e.Description, "\n", " ", -1)
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
{{if .Messages}}
// Message IDs
const ({{range .Messages}}
	MSG_ID_{{.Name}} mavlink.MessageID = {{.ID}}{{end}}
)
{{end}}

{{if .Messages}}
func init() { {{range .Messages}} 
	mavlink.Register(MSG_ID_{{.Name}}, "MSG_ID_{{.Name}}", {{.CRCExtra}}, func(p *mavlink.Packet) mavlink.Message {
		msg := new({{.Name | UpperCamelCase}})
		msg.Unpack(p)
		return msg
	}){{end}}
} {{end}}
`
	return template.Must(template.New("msgIds").Funcs(funcMap).Parse(msgIDTmpl)).Execute(w, d)
}

// generate class definitions for each msg id.
// for now, pack/unpack payloads via encoding/binary since it
// is expedient and correct. optimize this if/when needed.
func (d *Dialect) generateClasses(w io.Writer) error {
	classesTmpl := `
{{$mavlinkVersion := .MavlinkVersion}}
{{range .Messages}}
{{$name := .Name | UpperCamelCase}}
// {{$name}} struct (generated typeinfo)  
// {{.Description}}
type {{$name}} struct { {{range .Fields}}
  {{.Name | UpperCamelCase}} {{if .Enum}} {{.Enum}} {{.Tag}} {{ else }} {{.GoType}} {{ end }} // {{.Description}}{{end}}
}

// MsgID (generated function)
func (m *{{$name}}) MsgID() mavlink.MessageID {
	return MSG_ID_{{.Name}}
}

// String (generated function)
func (m *{{$name}}) String() string {
	return fmt.Sprintf(
		"&{{.DialectName}}.{{$name}}{ {{range $i, $v := .Fields}}{{if gt $i 0}}, {{end}}{{.Name | UpperCamelCase}}: {{if IsByteArrayField .}}%0X (\"%s\"){{else}}%+v{{end}}{{end}} }", 
		{{range .Fields}}m.{{.Name | UpperCamelCase}}{{if IsByteArrayField .}}, string(m.{{.Name | UpperCamelCase}}[:]){{end}},
{{end}}
	)
}

// Pack (generated function)
func (m *{{$name}}) Pack(p *mavlink.Packet) error {
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
func (m *{{$name}}) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < {{ .Size }} {
{{- if eq $mavlinkVersion 1 }}
		return mavlink.ErrPayloadTooSmall
{{- else }}
		payload = append(payload, mavlink.ZeroTail[:{{ .Size }}-len(p.Payload)]...)
{{- end }}
	}{{range .Fields}}
	{{.PayloadUnpackSequence}}{{end}}
	return nil
}
{{end}}
`
	for _, m := range d.Messages {
		m.Description = strings.Replace(m.Description, "\n", "\n// ", -1)
		if len(m.DialectName) == 0 {
			m.DialectName = baseName(d.FilePath)
		}
		for _, f := range m.Fields {
			f.Description = strings.Replace(f.Description, "\n", " ", -1)
			goname, gosz, golen, err := GoTypeInfo(f.CType)
			if err != nil {
				return err
			}
			f.GoType, f.BitSize, f.ArrayLen = goname, gosz, golen
			if len(f.Enum) > 0 {
				f.Tag = "`gotype:\"" + f.GoType + "\"`"
			}
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

func (d *Dialect) merge(rhs *Dialect) error {
	for _, enum := range rhs.Enums {
		if i := d.enumIdx(enum); i < 0 {
			d.Enums = append(d.Enums, enum)
		} else {
			for _, entry := range enum.Entries {
				if j := d.Enums[i].entryIdx(entry); j < 0 {
					d.Enums[i].Entries = append(d.Enums[i].Entries, entry)
				}
			}
		}
	}
	for _, message := range rhs.Messages {
		if i := d.messageIdx(message); i < 0 {
			message.DialectName = baseName(rhs.FilePath)
			d.Messages = append(d.Messages, message)
		}
	}
	return nil
}

func (d *Dialect) messageIdx(message *Message) int {
	for i, m := range d.Messages {
		if m.Name == message.Name {
			return i
		}
	}
	return -1
}

func (d *Dialect) enumIdx(enum *Enum) int {
	for i, e := range d.Enums {
		if e.Name == enum.Name {
			return i
		}
	}
	return -1
}

func (enum *Enum) entryIdx(entry *EnumEntry) int {
	for i, e := range enum.Entries {
		if e.Name == entry.Name {
			return i
		}
	}
	return -1
}
