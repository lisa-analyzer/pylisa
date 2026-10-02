package it.unive.pylisa.testnatives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.natives.Expressions;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;

/**
 * A callable that returns {@code None}.
 */
public class NoneNative extends LibraryNative {

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected NoneNative(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "testnatives.none", parameters);
	}

	/**
	 * Builds the model of one call; the factory the library loader uses.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 *
	 * @return the model
	 */
	public static NoneNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new NoneNative(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.returning(new Expressions(callSite()).none());
	}
}
