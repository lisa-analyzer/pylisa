package it.unive.pylisa;

import it.unive.pylisa.frontend.imports.ModuleImportDescriptor;

import java.util.Map;

public interface ImportDescriptor {
    public Map<String, ModuleImportDescriptor> getImportDescriptor();
}
