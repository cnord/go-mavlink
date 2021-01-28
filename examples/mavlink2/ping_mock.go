package mavlink

import (
	"encoding/binary"
	"fmt"
)

const (
	MSG_ID_PING_MOCK MessageID = 222
)

// ping struct (generated typeinfo)
// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://io/en/services/ping.html
type ping struct {
	Seq uint32 // ping sequence
}

// MsgID (generated function)
func (m *ping) MsgID() MessageID {
	return MSG_ID_PING_MOCK
}

// String (generated function)
func (m *ping) String() string {
	return fmt.Sprintf(
		"&ping{ Seq: %+v }",
		m.Seq,
	)
}

// Pack (generated function)
func (m *ping) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Seq))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ping) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, ZeroTail[:4-len(p.Payload)]...)
	}
	m.Seq = uint32(binary.LittleEndian.Uint32(payload[0:]))
	return nil
}
