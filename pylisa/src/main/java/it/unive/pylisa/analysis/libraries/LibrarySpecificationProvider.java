package it.unive.pylisa.analysis.libraries;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.*;
import it.unive.pylisa.analysis.libraries.pandas.ReadCsv;
import it.unive.pylisa.analysis.libraries.standardLibrary.Print;
import it.unive.pylisa.analysis.libraries.warnings.FilterWarnings;
import it.unive.pylisa.cfg.PythonUnit;
import it.unive.pylisa.cfg.type.PyLibraryType;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class LibrarySpecificationProvider {
    public static Collection<CompilationUnit> getLibraries() {
        Set<CompilationUnit> result = new HashSet<>();
        result.add(LibrarySpecificationProvider.getPandasPythonUnit());
        result.add(LibrarySpecificationProvider.getWarningsPythonUnit());
        return result;
    }

    public static Collection<NativeCFG> getAllStandardLibraryMethods(Unit unit) {
        Set<NativeCFG> result = new HashSet<>();
        result.add(new NativeCFG(
                new CFGDescriptor(new SourceCodeLocation("standard_library", 0, 0),
                        unit,
                        false,
                        "print",
                        new Parameter(new SourceCodeLocation("standard_library", 0, 0), "arg1"))
                , Print.class));
        return result;
    }

    private static PythonUnit getWarningsPythonUnit() {
        PythonUnit unit1 = new PythonUnit(new SourceCodeLocation("warnings", 0, 0), "warnings", true);
        NativeCFG cfg = new NativeCFG(
                new CFGDescriptor(new SourceCodeLocation("warnings", 0, 0),
                        unit1,
                        true,
                        "filterwarnings",
                        new Parameter(new SourceCodeLocation("warnings", 0, 0), "arg1", new PyLibraryType("warnings")),
                        new Parameter(new SourceCodeLocation("warnings", 0, 0), "arg2"))
                , FilterWarnings.class);
        unit1.addInstanceConstruct(cfg);
        return unit1;
    }

    private static PythonUnit getPandasPythonUnit() {
        PythonUnit unit1 = new PythonUnit(new SourceCodeLocation("pandas", 0, 0), "pandas", true);
        unit1.addInstanceConstruct(new NativeCFG(
                new CFGDescriptor(new SourceCodeLocation("pandas", 0, 0),
                        unit1,
                        true,
                        "read_csv",
                        new Parameter(new SourceCodeLocation("pandas", 0, 0), "arg1", new PyLibraryType("pandas")),
                        new Parameter(new SourceCodeLocation("pandas", 0, 0), "filepath_or_buffer"))
                , ReadCsv.class));
        return unit1;
    }
}
