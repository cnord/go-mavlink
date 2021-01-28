package main

import (
	"flag"
	"log"
)

var (
	mavgenVersion  = "devel"
	schemeFile     = flag.String("f", "", "mavlink xml-definition file input")
	version        = flag.String("v", mavgenVersion, "version of mavlink dialect")
	mavlinkVersion = flag.Int("m", 2, "version of mavlink protocol")
)

type templateData struct {
	Version        string
	MavlinkVersion int
}

func main() {
	log.SetFlags(0)
	log.SetPrefix("mavgen: ")
	flag.Parse()

	if err := generateDialect(*schemeFile, *mavlinkVersion); err != nil {
		log.Fatal(err)
	}

	data := templateData{
		Version:        *version,
		MavlinkVersion: *mavlinkVersion,
	}

	if err := generateCommons(data); err != nil {
		log.Fatal(err)
	}
}
