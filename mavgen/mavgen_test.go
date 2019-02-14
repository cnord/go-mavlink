package main

import (
	"fmt"
	"reflect"
	"strings"
	"testing"
)

func TestTypeConversions(t *testing.T) {

	cases := []struct {
		in              string
		name            string
		bitsz, arraylen int
	}{
		{"char", "byte", 8, 0},
		{"uint8_t", "uint8", 8, 0},
		{"uint16_t", "uint16", 16, 0},
		{"uint32_t", "uint32", 32, 0},
		{"uint64_t", "uint64", 64, 0},
		{"float", "float32", 32, 0},
		{"double", "float64", 64, 0},
		{"char[10]", "[10]byte", 8, 10},
		{"float[30]", "[30]float32", 32, 30},
	}

	for _, c := range cases {
		name, bitsz, arraylen, err := GoTypeInfo(c.in)
		// XXX: should test some cases that generate errors...
		if err != nil {
			t.Error("Type conversion err:", err)
		}
		if name != c.name {
			t.Errorf("Type Conversion for %q, got name %q, want %q", c.in, name, c.name)
		}
		if bitsz != c.bitsz {
			t.Errorf("Type Conversion for %q, got bitsz %q, want %q", c.in, bitsz, c.bitsz)
		}
		if arraylen != c.arraylen {
			t.Errorf("Type Conversion for %q, got arraylen %q, want %q", c.in, arraylen, c.arraylen)
		}
	}
}

func TestNameConversion(t *testing.T) {

	cases := []struct{ in, want string }{
		{"test", "Test"},
		{"_test_", "Test"},
		{"_test", "Test"},
		{"test_", "Test"},
		{"test_thing", "TestThing"},
		{"test_thing_", "TestThing"},
		{"TEST_THING", "TestThing"},
		{"_TEST_", "Test"},
		{"_TEST___THiNG__", "TestThing"},
		{"_TEST___THiNG_A__", "TestThingA"},
	}

	for _, c := range cases {
		got := UpperCamelCase(c.in)
		if got != c.want {
			t.Errorf("Upper Camel Conversion for %q, got %q, want %q", c.in, got, c.want)
		}
	}
}

type dialect struct {
	version  string
	enums    []*Enum
	messages []*Message
}

func dialectExtractMeaninful(d *Dialect) *dialect {
	enums := []*Enum{}
	if len(d.Enums) != 0 {
		enums = d.Enums
	}

	messages := []*Message{}
	if len(d.Messages) != 0 {
		messages = d.Messages
	}

	return &dialect{
		d.Version, enums, messages,
	}
}

func checkDialect(d1 *Dialect, d2 *Dialect) bool {
	return reflect.DeepEqual(*dialectExtractMeaninful(d1), *dialectExtractMeaninful(d2))
}

func TestParseDialect(t *testing.T) {

	enums := []*Enum{
		{"MAV_AUTOPILOT", "descr1", []*EnumEntry{
			{0, "MAV_AUTOPILOT_GENERIC", "", []*EnumEntryParam(nil)},
			{1, "MAV_AUTOPILOT_RESERVED", "", []*EnumEntryParam(nil)},
		}},
	}

	messages := []*Message{
		{1, "MSG1", "descr1", []*MessageField{
			{"uint32_t", "f1", "", "descr1", "", 0, 0, 0},
			{"uint8_t", "f2", "", "descr2", "", 0, 0, 0},
		}, ""},
		{2, "MSG2", "descr2", []*MessageField{
			{"uint8_t[10]", "f1", "", "descr1", "", 0, 0, 0},
		}, ""},
	}

	cases := []struct {
		name    string
		xml     string
		dialect *Dialect
		err     error
	}{
		{
			"empty",
			`<?xml version='1.0'?>
<mavlink>
    <version>3</version>
    <dialect>0</dialect>
</mavlink>`,
			&Dialect{Version: "3", Enums: []*Enum{}, Messages: []*Message{}},
			nil,
		},

		{
			"simple",
			`<?xml version='1.0'?>
<mavlink>
    <version>3</version>
    <dialect>0</dialect>
    <enums>
         <enum name="MAV_AUTOPILOT">
             <description>descr1</description>
	     <entry value="0" name="MAV_AUTOPILOT_GENERIC"></entry>
	     <entry value="1" name="MAV_AUTOPILOT_RESERVED"></entry>
         </enum>
    </enums>

    <messages>
        <message id="1" name="MSG1">
            <description>descr1</description>
            <field type="uint32_t" name="f1">descr1</field>
            <field type="uint8_t" name="f2">descr2</field>
        </message>
        <message id="2" name="MSG2">
            <description>descr2</description>
            <field type="uint8_t[10]" name="f1">descr1</field>
        </message>
    </messages>
</mavlink>`,
			&Dialect{Version: "3", Enums: enums, Messages: messages},
			nil,
		},

		{
			"mavlink-v2-message",
			`<?xml version='1.0'?>
<mavlink>
    <version>3</version>
    <dialect>0</dialect>
    <enums>
         <enum name="MAV_AUTOPILOT">
             <description>descr1</description>
	     <entry value="0" name="MAV_AUTOPILOT_GENERIC"></entry>
	     <entry value="1" name="MAV_AUTOPILOT_RESERVED"></entry>
         </enum>
    </enums>

    <messages>
        <message id="1" name="MSG1">
            <description>descr1</description>
            <field type="uint32_t" name="f1">descr1</field>
            <field type="uint8_t" name="f2">descr2</field>
        </message>
        <message id="2" name="MSG2">
            <description>descr2</description>
            <field type="uint8_t[10]" name="f1">descr1</field>
        </message>
        <message id="256" name="MSG1">
            <description>descr1</description>
            <field type="uint32_t" name="f1">descr1</field>
        </message>
    </messages>
</mavlink>`,
			&Dialect{Version: "3", Enums: enums, Messages: messages},
			nil,
		},

		{
			"v2-fields",
			`<?xml version='1.0'?>
<mavlink>
    <version>3</version>
    <dialect>0</dialect>
    <enums>
         <enum name="MAV_AUTOPILOT">
             <description>descr1</description>
	     <entry value="0" name="MAV_AUTOPILOT_GENERIC"></entry>
	     <entry value="1" name="MAV_AUTOPILOT_RESERVED"></entry>
         </enum>
    </enums>

    <messages>
        <message id="1" name="MSG1">
            <description>descr1</description>
            <field type="uint32_t" name="f1">descr1</field>
            <field type="uint8_t" name="f2">descr2</field>
            <extensions/>
            <field type="uint8_t" name="f3">descr3</field>
        </message>
        <message id="2" name="MSG2">
            <description>descr2</description>
            <field type="uint8_t[10]" name="f1">descr1</field>
            <extensions/>
        </message>
    </messages>
</mavlink>`,
			&Dialect{Version: "3", Enums: enums, Messages: messages},
			nil,
		},

		{
			"v2-fields-only",
			`<?xml version='1.0'?>
<mavlink>
    <version>3</version>
    <dialect>0</dialect>
    <enums>
         <enum name="MAV_AUTOPILOT">
             <description>descr1</description>
	     <entry value="0" name="MAV_AUTOPILOT_GENERIC"></entry>
	     <entry value="1" name="MAV_AUTOPILOT_RESERVED"></entry>
         </enum>
    </enums>

    <messages>
        <message id="1" name="MSG1">
            <extensions/>
            <description>descr1</description>
            <field type="uint32_t" name="f1">descr1</field>
            <field type="uint8_t" name="f2">descr2</field>
        </message>
        <message id="2" name="MSG2">
            <description>descr2</description>
            <extensions/>
        </message>
    </messages>
</mavlink>`,
			&Dialect{
				Version: "3", Enums: enums,
				Messages: []*Message{
					{1, "MSG1", "descr1", []*MessageField{}, ""},
					{2, "MSG2", "descr2", []*MessageField(nil), ""},
				},
			},
			nil,
		},
	}

	for ind, c := range cases {
		t.Run(fmt.Sprintf("%v.%v", ind, c.name), func(t *testing.T) {
			r := strings.NewReader(c.xml)
			d, err := ParseDialect(r, c.name)
			if err != c.err {
				t.Fatalf("expected err to be %+v, got %+v", c.err, err)
			}
			if !checkDialect(d, c.dialect) {
				t.Fatalf("expected to get dialect %+v, got %+v", c.dialect, d)
			}
		})
	}

}
