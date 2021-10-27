package it.unive.pylisa.analysis.libraries;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.pylisa.cfg.PythonUnit;

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

    private static PythonUnit getWarningsPythonUnit() {
        PythonUnit unit1 = new PythonUnit(new SourceCodeLocation("warnings", 0, 0), "warnings", true);
        NativeCFG cfg = new NativeCFG(
                new CFGDescriptor(new SourceCodeLocation("warnings", 0, 0),
                        unit1,
                        true,
                        "filterwarnings",
                        new Parameter(new SourceCodeLocation("warnings", 0, 0), "arg1"),
                        new Parameter(new SourceCodeLocation("warnings", 0, 0), "arg2"))
                , NoEffectMethod.FilterWarningsNoEffectMethod.class);
        unit1.addInstanceConstruct(cfg);
        return unit1;
    }

    private static PythonUnit getPandasPythonUnit() {
        PythonUnit unit1 = new PythonUnit(new SourceCodeLocation("pandas", 0, 0), "pandas", true);
        NativeCFG cfg = new NativeCFG(
                new CFGDescriptor(new SourceCodeLocation("pandas", 0, 0),
                        unit1,
                        true,
                        "read_csv",
                        new Parameter(new SourceCodeLocation("pandas", 0, 0), "arg1"),
                        new Parameter(new SourceCodeLocation("pandas", 0, 0), "filepath_or_buffer"))
                , ReturningFirstParameterExpressionNoSideEffectMethod.ReadCsvNoEffectMethod.class);
        unit1.addInstanceConstruct(cfg);
        return unit1;
    }
}
