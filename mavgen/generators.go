//go:generate templify register.template
//go:generate templify constants.template
//go:generate templify encoder.template
//go:generate templify decoder.template
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
		"register":     registerTemplate,
		"constants":    constantsTemplate,
		"encoder":      encoderTemplate,
		"decoder":      decoderTemplate,
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
	dir, err := os.Getwd()
	if err != nil {
		log.Fatal("Getwd(): ", err)
	}
	return filepath.Join(dir, baseName(scheme), "dialect.go")
}

func generateDialect(schemeFile string, mavlinkVersion int) (error) {
	d, err := ParseDialect(schemeFile, baseName(schemeFile))
	if err != nil {
		return err
	}

	d.MavlinkVersion = mavlinkVersion

	dialectFileName := findOutFile(schemeFile)

	if err = os.MkdirAll(filepath.Dir(dialectFileName), os.ModePerm); err != nil {
		return err
	}

	dialectFile, err := os.Create(dialectFileName)
	if err != nil {
		return err
	}
	defer dialectFile.Close()

	if err := d.GenerateGo(dialectFile); err != nil {
		return err
	}

	for _, i := range d.Include {
		includePath := filepath.Join(filepath.Dir(schemeFile), i)
		if err := generateDialect(includePath, mavlinkVersion); err != nil {
			return err
		}
	}

	return nil
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

func generateCommons(data templateData) error {
	cwd, err := os.Getwd()
	if err != nil {
		log.Fatal("Getwd(): ", err)
	}

	for k, v := range templates {
		if err := generateCode(cwd + string(filepath.Separator), data, k, v()); err != nil {
			return err
		}
	}
	return nil
}
