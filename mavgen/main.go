package main

import (
	"flag"
	"github.com/asmyasnikov/go-mavlink/common"
	"log"
)

var (
	mavgenVersion      = "devel"
	schemeFile         = flag.String("f", "", "mavlink xml-definition file input")
	version            = flag.String("v", mavgenVersion, "custom version of mavlink dialect")
	commonPackageFiles = flag.Bool("c", false, "generate common mavlink package code")
)

type templateData struct {
	Version        string
	MavlinkVersion int
}

func main() {
	log.SetFlags(0)
	log.SetPrefix("mavgen: ")
	flag.Parse()

	if len(*schemeFile) == 0 {
		flag.PrintDefaults()
		return
	}

	if err := generateDialect(nil, *schemeFile, common.MavlinkVersion()); err != nil {
		log.Fatal(err)
	}

	if *commonPackageFiles {
		data := templateData{
			Version:        *version,
			MavlinkVersion: common.MavlinkVersion(),
		}
		if err := generateCommons(data); err != nil {
			log.Fatal(err)
		}
	}
}
