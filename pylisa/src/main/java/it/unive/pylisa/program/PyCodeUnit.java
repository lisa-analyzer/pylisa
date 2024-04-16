package it.unive.pylisa.program;

import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CodeLocation;

public class PyCodeUnit extends CodeUnit {

    boolean mainUnit = false;

    /**
     * Builds a unit, defined at the given location.
     *
     * @param location the location where the unit is define within the source
     *                 file
     * @param program  the program where this unit is defined
     * @param name     the name of the unit
     */
    public PyCodeUnit(CodeLocation location, Program program, String name) {
        super(location, program, name);
    }

    /**
     * Builds a unit, defined at the given location.
     *
     * @param location the location where the unit is define within the source
     *                 file
     * @param program  the program where this unit is defined
     * @param name     the name of the unit
     * @param mainUnit whether or not this unit should be threated as a mainUnit
     */
    public PyCodeUnit(CodeLocation location, Program program, String name, boolean mainUnit) {
        super(location, program, name);
        this.mainUnit = mainUnit;
    }

    public boolean isMainUnit() {
        return this.mainUnit;
    }

}
