package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;

/**
 * The synthetic class of the closures of a nested function (or lambda) that
 * refers to variables of the enclosing functions (its free variables). A
 * closure is the value of the function: an object of this class, with a field
 * for each free variable, pointing to the object holding the cells of the
 * function that owns the variable (see
 * {@link it.unive.pylisa.cfg.expression.Cells}). Calling a closure calls the
 * function, passing the closure itself as its first (hidden) parameter.
 */
public class PyClosureUnit extends ClassUnit {

	private final CFG function;

	/**
	 * Builds the class of the closures of the given function.
	 *
	 * @param location the location of the function
	 * @param program  the program
	 * @param name     the name of the class
	 * @param function the function
	 */
	public PyClosureUnit(
			CodeLocation location,
			Program program,
			String name,
			CFG function) {
		super(location, program, name, true);
		this.function = function;
	}

	/**
	 * Yields the function called by the closures of this class.
	 *
	 * @return the function
	 */
	public CFG getFunction() {
		return function;
	}
}
