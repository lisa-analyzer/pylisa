package it.unive.pylisa.frontend.imports;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class ProjectModuleImports {

    private final Map<CodeLocation, ModuleImports> imports = new HashMap<>();

    public ProjectModuleImports() {}

    public ModuleImports getModuleImports(CodeLocation location) {
        return this.imports.get(location);
    }

    public void putModuleImports(CodeLocation location, ModuleImports mi) {
        this.imports.put(location, mi);
    }

    public ModuleImports[] getAllModuleImports() {
        return imports.entrySet().toArray(new ModuleImports[0]);
    }
    public void putModuleImports(CodeLocation location) {
        this.imports.put(location, new ModuleImports("$main"));
    }
}
