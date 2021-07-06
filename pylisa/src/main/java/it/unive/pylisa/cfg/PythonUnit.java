package it.unive.pylisa.cfg;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;

public class PythonUnit extends Unit
{
    /**
     * Builds a unit, defined at the given location.
     *
     * @param location the location where the unit is define within the source
     *                 file
     * @param name     the name of the unit
     */
    public PythonUnit(CodeLocation location, String name) {
        super(location, name);
    }
}
