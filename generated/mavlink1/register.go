/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import "strconv"

// Register method provide register dialect message on decoder knowledge
func Register(msgID MessageID, msgName string, crcExtra uint8, msgConstructor func(p *Packet) Message) {
	if exists, ok := msgNames[msgID]; ok {
		panic("Message with ID = " + strconv.Itoa(int(msgID)) + " already exists. Fix collision '" + msgName + "' vs '" + exists + "' and re-run mavgen")
	} else {
		msgNames[msgID] = "MSG_ID_SENSOR_OFFSETS"
		msgCrcExtras[msgID] = crcExtra
		msgConstructors[msgID] = msgConstructor
	}
}
