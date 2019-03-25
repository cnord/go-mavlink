/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import "sync"

// DialectSlice typedef is an alias for a slice of Dialect pointers
// Only really intended to be accessed as a field on Encoder/Decoder
type DialectSlice []*Dialect

var (
	mtx sync.Mutex
)

// look up the crcextra for msgid
func (ds *DialectSlice) findCrcX(msgid MessageID) (uint8, error) {
	mtx.Lock()
	defer mtx.Unlock()
	// http://www.mavlink.org/mavlink/crc_extra_calculation
	for _, d := range *ds {
		if crcx, ok := d.crcExtras[msgid]; ok {
			return crcx, nil
		}
	}

	return 0, ErrUnknownMsgID
}

func (ds *DialectSlice) indexOf(d *Dialect) int {
	for i, dlct := range *ds {
		if d.Name == dlct.Name {
			return i
		}
	}
	return -1
}

// IndexOf returns the index of d or -1 if not found
func (ds *DialectSlice) IndexOf(d *Dialect) int {
	mtx.Lock()
	defer mtx.Unlock()
	return ds.indexOf(d)
}

// Add appends d if not already present in ds
func (ds *DialectSlice) Add(d *Dialect) {
	mtx.Lock()
	defer mtx.Unlock()
	if ds.indexOf(d) < 0 {
		*ds = append(*ds, d)
	}
}

// Remove removes d if present in ds
func (ds *DialectSlice) Remove(d *Dialect) {
	mtx.Lock()
	defer mtx.Unlock()
	if i := ds.indexOf(d); i >= 0 {
		// https://github.com/golang/go/wiki/SliceTricks
		(*ds)[len(*ds)-1], *ds = nil, append((*ds)[:i], (*ds)[i+1:]...)
	}
}
