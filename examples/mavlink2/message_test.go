package mavlink

import (
	"bytes"
	"log"
	"reflect"
	"testing"
)

func TestRoundTrip(t *testing.T) {

	cases := []struct{ seq uint32 }{
		{12345},
	}

	for _, c := range cases {
		p := CommonPing{
			Seq: c.seq,
		}

		var pkt Packet
		if err := p.Pack(&pkt); err != nil {
			t.Errorf("Pack fail %q (%q)", pkt, err)
		}

		var buf bytes.Buffer

		if err := NewEncoder(&buf).EncodePacket(&pkt); err != nil {
			t.Errorf("Encode fail %q", err)
		} else {
			log.Println("Good encode")
		}

		pktOut, err := NewDecoder(&buf).Decode()
		if err != nil {
			t.Errorf("Decode fail %q", err)
		} else {
			log.Println("Good decode")
		}

		if pktOut.MsgID != MSG_ID_PING {
			t.Errorf("MsgID fail, want %q, got %q", MSG_ID_PING, pktOut.MsgID)
		}

		var pingOut CommonPing
		if err := pingOut.Unpack(pktOut); err != nil {
			t.Errorf("Unpack fail %q", err)
		}

		if pingOut.Seq != c.seq {
			t.Errorf("Mismatch msg field, got %q, want %q", pingOut.Seq, c.seq)
		}
	}
}

func TestRoundTripChannels(t *testing.T) {

	data := make(chan []byte, 256)
	enc := NewChannelEncoder(data)
	dec := NewChannelDecoder(data)

	cases := []struct{ seq uint32 }{
		{12345},
	}

	for _, c := range cases {
		p := CommonPing{
			Seq: c.seq,
		}

		var pkt Packet
		if err := p.Pack(&pkt); err != nil {
			t.Errorf("Pack fail %q (%q)", pkt, err)
		}

		if err := enc.EncodePacket(&pkt); err != nil {
			t.Errorf("Encode fail %q", err)
		} else {
			log.Println("Good encode")
		}

		pktOut, err := dec.Decode()
		if err != nil {
			t.Errorf("Decode fail %q", err)
		} else {
			log.Println("Good decode")
		}

		if pktOut.MsgID != MSG_ID_PING {
			t.Errorf("MsgID fail, want %q, got %q", MSG_ID_PING, pktOut.MsgID)
		}

		var pingOut CommonPing
		if err := pingOut.Unpack(pktOut); err != nil {
			t.Errorf("Unpack fail %q", err)
		}

		if pingOut.Seq != c.seq {
			t.Errorf("Mismatch msg field, got %q, want %q", pingOut.Seq, c.seq)
		}
	}
}

func TestDecode(t *testing.T) {
	// decode a known good byte stream
	pktbytes := []byte{0xFD, 0x2B, 0x00, 0x00, 0xC5, 0x02, 0xFC, 0x64, 0x00, 0x00, 0x16, 0x89, 0xB6, 0x44, 0xA5, 0x47, 0x00, 0x00, 0xAB, 0x47, 0x00, 0x00, 0xAA, 0x47, 0x00, 0x00, 0xBF, 0x04, 0x00, 0x00, 0xA4, 0xBD, 0x02, 0x31, 0x2E, 0x30, 0x30, 0x00, 0x05, 0x11, 0x12, 0x0A, 0x01, 0x00, 0x64, 0x00, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x02, 0x4A, 0x26}
	_, err := NewDecoder(bytes.NewBuffer(pktbytes)).Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}
}

func TestDecodeTwoMessages(t *testing.T) {
	// decode a known good byte stream
	pktbytes := []byte{0xFD, 0x0D, 0x00, 0x00, 0x8A, 0x02, 0xFC, 0x04, 0x00, 0x00, 0xE5, 0xF2, 0x41, 0x21, 0x09, 0x7E, 0x3B, 0x16, 0x00, 0x00, 0x01, 0x00, 0x6B, 0x4A, 0x83,
		0xfe, 0x0e, 0x00, 0xFD, 0x2B, 0x00, 0x00, 0xC5, 0x02, 0xFC, 0x64, 0x00, 0x00, 0x16, 0x89, 0xB6, 0x44, 0xA5, 0x47, 0x00, 0x00, 0xAB, 0x47, 0x00, 0x00, 0xAA, 0x47, 0x00, 0x00, 0xBF, 0x04, 0x00, 0x00, 0xA4, 0xBD, 0x02, 0x31, 0x2E, 0x30, 0x30, 0x00, 0x05, 0x11, 0x12, 0x0A, 0x01, 0x00, 0x64, 0x00, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x02, 0x4A, 0x26}
	decoder := NewDecoder(bytes.NewBuffer(pktbytes))

	msg1, err := decoder.Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}

	msg2, err := decoder.Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}

	if reflect.DeepEqual(msg1, msg2) {
		t.Error("Messages should not match")
	}
}

func TestDecodeFalseStart(t *testing.T) {
	// a false start followed by a known good byte stream
	pktbytes := []byte{0xFD, 0x0D, 0xFD, 0x0D, 0xFD, 0x0D, 0x00, 0x00, 0x8A, 0x02, 0xFC, 0x04, 0x00, 0x00, 0xE5, 0xF2, 0x41, 0x21, 0x09, 0x7E, 0x3B, 0x16, 0x00, 0x00, 0x01, 0x00, 0x6B, 0x4A, 0x83}
	_, err := NewDecoder(bytes.NewBuffer(pktbytes)).Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}
}

func TestDecodeMultipleFalseStarts(t *testing.T) {
	// two false starts followed by a known good byte stream
	pktbytes := []byte{0xFD, 0x0D, 0xFD, 0x0D, 0xFD, 0x0D, 0xFD, 0x0D, 0x00, 0x00, 0x8A, 0x02, 0xFC, 0x04, 0x00, 0x00, 0xE5, 0xF2, 0x41, 0x21, 0x09, 0x7E, 0x3B, 0x16, 0x00, 0x00, 0x01, 0x00, 0x6B, 0x4A, 0x83}

	decoder := NewDecoder(bytes.NewBuffer(pktbytes))
	_, err := decoder.Decode()
	// we can't trust message id while stream is corrupted
	for err == ErrUnknownMsgID {
		_, err = decoder.Decode()
	}
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}

	pktbytes = append([]byte{0x00, 0xFD, 0x2B, 0x00, 0x00, 0xC5, 0x02, 0xFC, 0x64, 0x00, 0x00, 0x16, 0x89, 0xB6, 0x44, 0xA5, 0x47, 0x00, 0x00, 0xAB, 0x47, 0x00, 0x00, 0xAA, 0x47, 0x00, 0x00, 0xBF, 0x04, 0x00, 0x00, 0xA4, 0xBD, 0x02, 0x31, 0x2E, 0x30, 0x30, 0x00, 0x05, 0x11, 0x12, 0x0A, 0x01, 0x00, 0x64, 0x00, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x02, 0x4A, 0x26}, pktbytes[:]...)
	decoder = NewDecoder(bytes.NewBuffer(pktbytes))
	_, err = decoder.Decode()
	// we can't trust message id while stream is corrupted
	for err == ErrUnknownMsgID {
		_, err = decoder.Decode()
	}
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}
}

func TestDecodeFalseStartTwoMessages(t *testing.T) {
	// a false start followed by a known good byte stream followed by another false start followed by a known byte stream
	pktbytes := []byte{0xFD, 0x00, 0xFD, 0x0D, 0x00, 0x00, 0x8A, 0x02, 0xFC, 0x04, 0x00, 0x00, 0xE5, 0xF2, 0x41, 0x21, 0x09, 0x7E, 0x3B, 0x16, 0x00, 0x00, 0x01, 0x00, 0x6B, 0x4A, 0x83,
		0xFD, 0xAB, 0xFD, 0x2B, 0xFD, 0x2B, 0x00, 0x00, 0xC5, 0x02, 0xFC, 0x64, 0x00, 0x00, 0x16, 0x89, 0xB6, 0x44, 0xA5, 0x47, 0x00, 0x00, 0xAB, 0x47, 0x00, 0x00, 0xAA, 0x47, 0x00, 0x00, 0xBF, 0x04, 0x00, 0x00, 0xA4, 0xBD, 0x02, 0x31, 0x2E, 0x30, 0x30, 0x00, 0x05, 0x11, 0x12, 0x0A, 0x01, 0x00, 0x64, 0x00, 0x00, 0x03, 0x07, 0x01, 0x00, 0x00, 0x02, 0x4A, 0x26}
	decoder := NewDecoder(bytes.NewBuffer(pktbytes))

	msg1, err := decoder.Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}

	msg2, err := decoder.Decode()
	if err != nil {
		t.Errorf("Decode fail: %s\n", err)
	}

	if reflect.DeepEqual(msg1, msg2) {
		t.Error("Messages should not match")
	}
}

func TestDialects(t *testing.T) {

	var buf bytes.Buffer

	enc := NewEncoder(&buf)
	dec := NewDecoder(&buf)

	// try to encode an ardupilot msg before we've added that dialect,
	// ensure it fails as expected
	mi := &ArdupilotmegaMeminfo{
		Brkval:  1000,
		Freemem: 10,
	}

	err := enc.Encode(0x1, 0x1, mi)
	if err != ErrUnknownMsgID {
		t.Errorf("encode expected ErrUnknownMsgID, got %q", err)
	}

	buf.Reset()

	if err = enc.Encode(0x1, 0x1, mi); err != nil {
		t.Errorf("Encode fail %q", err)
	}

	log.Println("TRY DECODE ARDUPILOT MESSAGE")
	_, err = NewDecoder(&buf).Decode()
	log.Println("ARDUPILOT MESSAGE DECODED")
	if err != ErrUnknownMsgID {
		t.Errorf("decode expected ErrUnknownMsgID, got %q", err)
	}


	// add the dialect, and ensure it succeeds
	dec.Dialects.Add(DialectArdupilotmega)
	// re-encode the msg, and decode it again after adding the required dialect
	if err = enc.Encode(0x1, 0x1, mi); err != nil {
		t.Errorf("Encode fail %q", err)
	}

	pktOut, err := dec.Decode()
	if err != nil {
		t.Errorf("Decode fail %q", err)
	}

	// make sure the output matches our original input for good measure
	var miOut ArdupilotmegaMeminfo
	if err := miOut.Unpack(pktOut); err != nil {
		t.Errorf("Unpack fail %q", err)
	}

	if miOut.Brkval != mi.Brkval || miOut.Freemem != mi.Freemem {
		t.Errorf("Round trip fail")
	}
}
