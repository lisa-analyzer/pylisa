package it.unive.pylisa.cfg.statement;

import it.unive.lisa.cfg.CFG;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;

public class FromImport extends Import {
    private String component;
    public FromImport(String importedLibrary, String component, String asName, CFG cfg, String sourceFile, int line, int col) {
        super(importedLibrary, asName, cfg, sourceFile, line, col);
        this.component = component;
    }

    @Override
    public String toString() {
        return "from "+super.importedLibrary+" import "+ component+" as "+super.name;
    }
}
