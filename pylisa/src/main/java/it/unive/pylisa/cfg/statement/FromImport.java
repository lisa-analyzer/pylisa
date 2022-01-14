package it.unive.pylisa.cfg.statement;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;

public class FromImport extends Import {
	private final String component;

	public FromImport(String importedLibrary, String component, String asName, CFG cfg, CodeLocation loc) {
		super(importedLibrary, asName, cfg, loc);
		this.component = component;
	}

	@Override
	public String toString() {
		return "from " + super.importedLibrary + " import " + component + " as " + super.name;
	}
}
