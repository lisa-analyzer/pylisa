package it.unive.pylisa.frontend.imports;

import jdk.jshell.spi.ExecutionControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ModuleImports {
    private final String scopeName;

    private final ModuleImports superScope;
    private final Map<String, ModuleImportDescriptor> imports = new HashMap<>();

    private final Map<String, ModuleImports> subScopes = new HashMap<>();

    public ModuleImports(String scopeName) {
        this.scopeName = scopeName;
        this.superScope = null;
    }

    public ModuleImports(String scopeName, ModuleImports superScope) {
        this.scopeName = scopeName;
        this.superScope = superScope;
    }

    public void addModuleImportDescriptor(ModuleImportDescriptor descriptor) {
        imports.put(descriptor.getAs(), descriptor);
    }

    public void changeAlias(String alias, String newAlias) {
        ModuleImportDescriptor mid = imports.get(alias);
        if (mid != null) {
            mid.setAs(newAlias);
            imports.remove(alias);
            imports.put(newAlias, mid); // this will remove the previous "import something as newAlias" ofc (correct behavior)
        }
    }
    public boolean hasImport(String key) {
        return imports.get(key) != null;
    }

    public ModuleImportDescriptor getModuleImportDescriptor(String alias) {
        return imports.get(alias);
    }


    public void removeModuleImportDescriptor(String alias) {
        imports.remove(alias);
    }

    public ModuleImportDescriptor[] getAllModuleImportDescriptors() {
        return imports.values().toArray(new ModuleImportDescriptor[0]);
    }

}
