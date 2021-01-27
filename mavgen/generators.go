//go:generate templify constants.template
//go:generate templify encoder.template
//go:generate templify decoder.template
//go:generate templify dialect.template
//go:generate templify message.template
//go:generate templify packet.template
//go:generate templify parser.template
//go:generate templify version.template
//go:generate templify x25.template

package main

import (
	"bytes"
	"errors"
	"fmt"
	"github.com/iancoleman/strcase"
	"go/format"
	"log"
	"os"
	"path/filepath"
	"strings"
	"text/template"
)

const (
	generatedHeader = "/*\n" +
		" * CODE GENERATED AUTOMATICALLY WITH\n" +
		" *    github.com/asmyasnikov/go-mavlink/mavgen\n" +
		" * THIS FILE SHOULD NOT BE EDITED BY HAND\n" +
		" */\n" +
		"\n"
)

var (
	templates = map[string](func() string){
		"constants":    constantsTemplate,
		"encoder":      encoderTemplate,
		"decoder":      decoderTemplate,
		"dialect":      dialectTemplate,
		"message":      messageTemplate,
		"packet":       packetTemplate,
		"parser":       parserTemplate,
		"version":      versionTemplate,
		"x25":          x25Template,
	}
)

// helper to remove the extension from the base name
func baseName(s string) string {
	return strings.TrimSuffix(filepath.Base(s), filepath.Ext(s))
}

func findOutFile(scheme string) string {
	if *outfile == "" {
		*outfile = baseName(scheme) + ".go"
	}

	dir, err := os.Getwd()
	if err != nil {
		log.Fatal("Getwd(): ", err)
	}

	return filepath.Join(dir, *outfile)
}

func generateDialect(schemeFile string, mavlinkVersion int) (*string, *string, error) {
	d, err := ParseDialect(schemeFile, baseName(schemeFile))
	if err != nil {
		return nil, nil, err
	}

	d.MavlinkVersion = mavlinkVersion

	dialectFileName := findOutFile(schemeFile)

	dialectFile, err := os.Create(dialectFileName)
	if err != nil {
		return nil, nil, err
	}
	defer dialectFile.Close()

	if err := d.GenerateGo(dialectFile); err != nil {
		return nil, nil, err
	}

	dialectName := "Dialect" + strcase.ToCamel(d.Name)

	return &dialectFileName, &dialectName, nil
}

func generateCode(dialectDir string, data templateData, templateName string, tmpl string) error {
	t, err := template.New(templateName).Parse(tmpl)

	if err != nil {
		return err
	}

	file, err := os.Create(dialectDir + templateName + ".go")
	if err != nil {
		return err
	}
	defer file.Close()

	n, err := file.Write([]byte(generatedHeader))
	if err != nil {
		return err
	} else if n < len(generatedHeader) {
		return errors.New("couldn't write NO-EDIT header")
	}

	var buffer bytes.Buffer
	if err := t.Execute(&buffer, data); err != nil {
		return err
	}
	formatted, err := format.Source(buffer.Bytes())
	if err != nil {
		log.Fatal("couldn't format generated "+templateName+".go: ", err)
		fmt.Print(buffer.Bytes())
		formatted = buffer.Bytes()
	}
	n, err = file.Write(formatted)
	if err != nil {
		return err
	} else if n < len(formatted) {
		return errors.New("couldn't write body of " + templateName + ".go")
	}
	return nil
}

func generateCommons(dialectDir string, data templateData) error {
	for k, v := range templates {
		if err := generateCode(dialectDir, data, k, v()); err != nil {
			return err
		}
	}
	return nil
}
