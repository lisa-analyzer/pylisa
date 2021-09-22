package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyLibraryType;

public class LibraryIdentifier extends Constant {
    /**
     * Builds the constant.
     *
     * @param name    the name of the library
     * @param location the code location of the statement that has generated
     */
    public LibraryIdentifier(String name, CodeLocation location) {
        super(new PyLibraryType(name), name, location);
    }
}
