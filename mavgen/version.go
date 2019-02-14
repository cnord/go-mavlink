/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/wlbr/templify
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package main

// versionTemplate is a generated function returning the template as a string.
// That string should be parsed by the functions of the golang's template package.
func versionTemplate() string {
	var tmpl = "package mavlink\n" +
		"\n" +
		"const (\n" +
		"    // Version of mavgen which generate this code or user defined (by mavgen flag -v) version of dialect\n" +
		"    Version        = \"{{- .Version -}}\"\n" +
		"    // MavlinkVersion is a user defined (by mavgen flag -m) version of mavlink protcol\n" +
		"    MavlinkVersion = {{- .MavlinkVersion -}}\n" +
		")"
	return tmpl
}
