package it.unive.pylisa;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;

import java.util.HashMap;
import java.util.Map;

public class PythonModuleImportManager {

    private final Program program;
    private final Map<String, ModuleUnit> loadedModules = new HashMap<>();
    private CFG init;
    public PythonModuleImportManager(Program program, CFG init) {
        this.program = program;
        this.init = init;
    }

    public ModuleUnit importModule(String moduleName) {
        if (loadedModules.containsKey(moduleName))
            return loadedModules.get(moduleName);

        ModuleUnit unit = null;

        if (LibrarySpecificationProvider.getLibraryUnit(moduleName) != null)
            unit = LibrarySpecificationProvider.importPythonModule(program, moduleName, init);
        if (unit == null && projectFileExists(moduleName)) {
            unit = loadProjectModule(moduleName);
        }
        if (unit == null) {
            unit = createSyntheticModule(moduleName);
        }

        // register in PyModuleType and cache
        PyModuleType.register(moduleName, unit);
        loadedModules.put(moduleName, unit);
        return unit;
    }

    private boolean projectFileExists(String moduleName) {
        // TODO: check if there's a .py file for moduleName in the project
        return false;
    }

    private ModuleUnit loadProjectModule(String moduleName) {
        // TODO: call front-end visitor / parser to load the file into a PythonModuleUnit
        return null;
    }

    private ModuleUnit createSyntheticModule(String moduleName) {
        ModuleUnit unit = new ModuleUnit(
                SyntheticLocation.INSTANCE, program, moduleName);
        PyCFG initModule = new PyCFG(
                new CodeMemberDescriptor(SyntheticLocation.INSTANCE, unit, false, "__initmodule__")
        );
       //initModule.addNode(new NoOp(initModule, SyntheticLocation.INSTANCE));
        unit.addCodeMember(initModule);

        program.addUnit(unit);
        return unit;
    }
}
