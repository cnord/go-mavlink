package mavlink

import (
	"reflect"
	"testing"
	"time"
)

func TestRoundTripChannels(t *testing.T) {

	AddDialect(DialectArdupilotmega)

	var data = make(chan []byte, 256)

	cases := []struct{ seq uint32 }{
		{12345},
		{54321},
		{65432},
		{23456},
		{65456},
		{23423},
		{65444},
		{23456},
		{76543},
	}

	dec := NewChannelDecoder(data)

	for i, _ := range cases {
		seq := cases[i].seq
		p := CommonPing{
			Seq: seq,
		}

		var packet Packet
		if err := p.Pack(&packet); err != nil {
			t.Errorf("Pack fail %q (%q)", packet, err)
		}
		packet.fixChecksum(dialects)

		data <- packet.Bytes()

		select {
		case packet := <- dec.decoded :
			if packet.MsgID != MSG_ID_PING {
				t.Errorf("MsgID fail, want %d, got %d", MSG_ID_PING, packet.MsgID)
			}

			var pingOut CommonPing
			if err := pingOut.Unpack(packet); err != nil {
				t.Errorf("Unpack fail %s", err)
			}

			if seq != pingOut.Seq {
				cases = append(cases[:i], cases[i+1:]...)
				break
			}
		case <-time.After(time.Millisecond) :
			t.Error("Timeout elapsed")
		}
	}
}

func TestDecode(t *testing.T) {
	// decode a known good byte stream
	var data = make(chan []byte)
	dec := NewChannelDecoder(data)
	data <- []byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E}
	select {
	case <- dec.decoded :
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
}

func TestDecodeTwoMessages(t *testing.T) {
	// decode a known good byte stream
	var data = make(chan []byte)
	dec := NewChannelDecoder(data)
	data <- []byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40}
	var msg1 *Packet
	select {
	case packet := <- dec.decoded :
		msg1 = packet
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
	var msg2 *Packet
	select {
	case packet := <- dec.decoded :
		msg2 = packet
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
	if reflect.DeepEqual(*msg1, *msg2) {
		t.Error("Messages should not match")
	}
}

func TestDecodeFalseStart(t *testing.T) {
	// a false start followed by a known good byte stream
	var data = make(chan []byte)
	dec := NewChannelDecoder(data)
	data <- []byte{0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E}
	select {
	case <- dec.decoded :
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
}

func TestDecodeMultipleFalseStarts(t *testing.T) {
	// two false starts followed by a known good byte stream
	var data = make(chan []byte)
	dec := NewChannelDecoder(data)
	data <- []byte{0xfe, 0x00, 0xfe, 0x00, 0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E,
		0xfe, 0x0e, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x45, 0x40}
	select {
	case msg1 := <- dec.decoded :
		select {
		case msg2 := <- dec.decoded :
			if reflect.DeepEqual(*msg1, *msg2) {
				t.Error("Messages should not match")
			}
		case <-time.After(time.Millisecond) :
			t.Error("Decode fail")
		}
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
}

func TestDialects(t *testing.T) {
	data := make(chan []byte, 256)

	dec := NewChannelDecoder(data)

	// try to encode an ardupilot msg before we've added that dialect,
	// ensure it fails as expected
	mi := &ArdupilotmegaMeminfo{
		Brkval:  1000,
		Freemem: 10,
	}

	RemoveDialect(DialectArdupilotmega)

	var packet Packet
	if err := packet.Encode(0x1, 0x1, mi); err != ErrUnknownMsgID {
		t.Errorf("encode expected ErrUnknownMsgID, got %s", err)
	}

	// add the dialect, and ensure it succeeds
	AddDialect(DialectArdupilotmega)

	if err := packet.Encode(0x1, 0x1, mi); err != nil {
		t.Errorf("encode unexpected err %s", err)
	}

	data <- packet.Bytes()

	select {
	case packet := <- dec.decoded :
		// make sure the output matches our original input for good measure
		var miOut ArdupilotmegaMeminfo
		if err := miOut.Unpack(packet); err != nil {
			t.Errorf("Unpack fail %s", err)
		}

		if miOut.Brkval != mi.Brkval || miOut.Freemem != mi.Freemem {
			t.Errorf("Round trip fail")
		}
	case <-time.After(time.Millisecond) :
		t.Error("Decode fail")
	}
}
