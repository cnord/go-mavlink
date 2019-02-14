package x25

// X25 implements hash.Hash
type X25 struct {
	crc uint16
}

// New function create crc calculator
func New() *X25 {
	x := &X25{}
	x.Reset()
	return x
}

// WriteByte function append byte into crc accumulation
func (x *X25) WriteByte(b byte) error {
	tmp := b ^ byte(x.crc&0xff)
	tmp ^= (tmp << 4)
	x.crc = (x.crc >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
	return nil
}

// Write function append byte slice into crc accumulation
func (x *X25) Write(p []byte) (n int, err error) {
	for i, b := range p {
		err := x.WriteByte(b)
		if err != nil {
			return i - 1, err
		}
	}
	return len(p), nil
}

// Sum16 function return accumulated crc
func (x *X25) Sum16() uint16 { return x.crc }

// Sum function append crc bytes on byte slice
func (x *X25) Sum(in []byte) []byte {
	s := x.Sum16()
	return append(in, byte(s>>8), byte(s))
}

// Size function return crc size in bytes
func (x *X25) Size() int { return 2 }

// BlockSize function return crc size in blocks
func (x *X25) BlockSize() int { return 1 }

// Reset function reset current crc
func (x *X25) Reset() { x.crc = 0xffff }
