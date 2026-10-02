package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import java.util.Objects;

/**
 * The code of a callable declared in a library specification: a native CFG
 * that also records the library declaring the callable and the Java class
 * modelling it. Readers of the analysis results use them to tell which library
 * a call reaches and whether it is modelled, without matching names.
 */
public final class LibraryNativeCFG extends NativeCFG {

	private final Class<? extends NaryExpression> implementation;

	private final String library;

	/**
	 * Builds the code of a library callable.
	 *
	 * @param descriptor     the descriptor of the callable
	 * @param implementation the class modelling the callable
	 * @param library        the name of the library declaring the callable,
	 *                           as written in its specification
	 */
	public LibraryNativeCFG(
			CodeMemberDescriptor descriptor,
			Class<? extends NaryExpression> implementation,
			String library) {
		super(descriptor, implementation);
		this.implementation = implementation;
		this.library = Objects.requireNonNull(library);
	}

	/**
	 * Yields the class modelling the callable.
	 *
	 * @return the class
	 */
	public Class<? extends NaryExpression> getImplementation() {
		return implementation;
	}

	/**
	 * Yields the name of the library declaring the callable.
	 *
	 * @return the library name
	 */
	public String getLibrary() {
		return library;
	}
}
