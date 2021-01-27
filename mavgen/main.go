package main

import (
	"flag"
	"log"
	"path/filepath"
	"strconv"
)

var (
	mavgenVersion  = "devel"
	schemeFile     = flag.String("f", "", "mavlink xml-definition file input")
	outfile        = flag.String("o", "", "output file name")
	version        = flag.String("v", mavgenVersion, "version of mavlink dialect")
	mavlinkVersion = flag.Int("m", 2, "version of mavlink protocol, usage with -p flag.")
)

type templateData struct {
	Version        string
	MavlinkVersion string
	Mavlink2       bool
	Mavlink1       bool
	DialectName    string
}

func main() {
	log.SetFlags(0)
	log.SetPrefix("mavgen: ")
	flag.Parse()

	dialectFileName, dialectName, err := generateDialect(*schemeFile, *mavlinkVersion)
	if err != nil {
		log.Fatal(err)
	}

	dialectDir := filepath.Dir(*dialectFileName) + string(filepath.Separator)

	data := templateData{
		Version:        *version,
		MavlinkVersion: strconv.Itoa(*mavlinkVersion),
		Mavlink2:       *mavlinkVersion&(1<<1) > 0,
		Mavlink1:       *mavlinkVersion&(1<<0) > 0,
		DialectName:    *dialectName,
	}

	if err := generateCommons(dialectDir, data); err != nil {
		log.Fatal(err)
	}
}
