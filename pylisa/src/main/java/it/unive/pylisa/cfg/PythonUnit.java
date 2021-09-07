package it.unive.pylisa.cfg;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;

public class PythonUnit extends Unit
{
    /**
     * Builds a unit, defined at the given location.
     *
     * @param name     the name of the unit
     */
    public PythonUnit(String name) {
        super(name);
    }
}
