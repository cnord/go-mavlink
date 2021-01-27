package main

import (
	"flag"
	"log"
	"path/filepath"
)

var (
	mavgenVersion  = "devel"
	schemeFile     = flag.String("f", "", "mavlink xml-definition file input")
	version        = flag.String("v", mavgenVersion, "version of mavlink dialect")
	mavlinkVersion = flag.Int("m", 2, "version of mavlink protocol, usage with -p flag.")
)

type templateData struct {
	Version        string
	MavlinkVersion int
}

func main() {
	log.SetFlags(0)
	log.SetPrefix("mavgen: ")
	flag.Parse()

	dialectFileName, err := generateDialect(*schemeFile, *mavlinkVersion)
	if err != nil {
		log.Fatal(err)
	}

	dialectDir := filepath.Dir(*dialectFileName) + string(filepath.Separator)

	data := templateData{
		Version:        *version,
		MavlinkVersion: *mavlinkVersion,
	}

	if err := generateCommons(dialectDir, data); err != nil {
		log.Fatal(err)
	}
}
