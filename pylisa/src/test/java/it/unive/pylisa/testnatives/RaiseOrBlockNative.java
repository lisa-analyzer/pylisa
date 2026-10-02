package it.unive.pylisa.testnatives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.type.PyExceptionType;
import it.unive.pylisa.libraries.natives.Diverges;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;

/**
 * A callable that either raises or never returns, and declares it, as an
 * event loop that runs until it fails.
 */
@Diverges
public class RaiseOrBlockNative extends LibraryNative {

	/**
	 * Builds the call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments
	 */
	protected RaiseOrBlockNative(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "testnatives.raise_or_block", parameters);
	}

	/**
	 * Builds the call, as the library loader does.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments
	 *
	 * @return the call
	 */
	public static RaiseOrBlockNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new RaiseOrBlockNative(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.raise(PyExceptionType.VALUE_ERROR);
	}
}
