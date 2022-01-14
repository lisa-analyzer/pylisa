package it.unive.pylisa.cfg;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CodeLocation;

public class PythonUnit extends CompilationUnit
{
    /**
     * Builds a compilation unit, defined at the given program point.
     *
     * @param location the location where the unit is define within the source
     *                 file
     * @param name     the name of the unit
     * @param sealed   whether or not this unit is sealed, meaning that it
     *                 cannot be used as super unit of other compilation
     */
    public PythonUnit(CodeLocation location, String name, boolean sealed) {
        super(location, name, sealed);
    }
}
