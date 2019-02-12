# go-mavlink

a [Go](http://golang.org/) package for reading/writing [mavlink](http://qgroundcontrol.org/mavlink/start) 1.0 and 2.0 messages.

## installation

* grab the files: `go get github.com/asmyasnikov/go-mavlink`
* generate classes from mavlink xml definitions: `go generate`

xml definitions can be updated from https://github.com/mavlink/mavlink

## usage

package `go-mavlink/mavgen` is used only for generating classes from mavlink xml definitions, it is not generally intended to be used by anything other than the `go generate` command.

package `go-mavlink/mavlink` is the main package used for encoding/decoding mavlink messages.

`mavlink.Packet` is the struct that gets sent over the wire, and `mavlink.Message` is an interface implemented by all mavlink classes generated by `mavgen`. 

### dialects

Several mavlink dialects define messages for overlapping msg ids, so it is required to specify which dialects you'd like to use. `DialectCommon` is included by most of the available dialects, so is included by default.

```go
dec := mavlink.NewDecoder(rdr)
dec.Dialects.Add(DialectArdupilotmega)
```

Existing dialects are:
* DialectArdupilotmega
* DialectAsluav
* DialectCommon (added to Encoders/Decoders by default, can be removed if desired)
* DialectMatrixpilot
* DialectPixhawk
* DialectUalberta

### decode

a typical decode loop might look like:

```go
rdr := SomeIoReader() // any io.Reader: UDP, serial port, bytes.Buffer, etc
dec := mavlink.NewDecoder(rdr)

for {
    pkt, err := dec.Decode()
    if err != nil {
        log.Fatal("Decode fail:", err)
    }

    // handle packet types you're interested in...
    switch pkt.MsgID {
    case mavlink.MSG_ID_PARAM_VALUE:
        var pv mavlink.ParamValue
        if err := pv.Unpack(pkt); err == nil {
            // handle param value
        }
    }
}
```

### encode

the most convenient way to encode is to use `Encoder.Encode()`:

```go
w := SomeIoWriter() // any io.Writer: UDP, serial port, bytes.Buffer, etc
enc := mavlink.NewEncoder(w)

p := mavlink.Ping{
    Seq: 12345,
}

if err := enc.Encode(0x1, 0x1, &p); err != nil {
    log.Fatal("Encode fail:", err)
}
```

if for some reason you need to pass a `Packet` directly, use `Encoder.EncodePacket()`:

```go
p := mavlink.Ping{
    Seq: c.seq,
}

var pkt Packet
if err := p.Pack(&pkt); err != nil {
    log.Fatal("Pack fail:", err)
}

if err := enc.EncodePacket(&pkt); err != nil {
    log.Fatal("Encode fail:", err)
}
```
