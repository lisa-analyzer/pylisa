package it.unive.pylisa.testnatives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * A callable that returns its argument and counts how many times its model is
 * applied, so that tests can check how often the analysis evaluates an
 * argument.
 */
public class TickNative extends LibraryNative {

	/**
	 * How many times the model has been applied since the last reset.
	 */
	public static final AtomicInteger APPLIED = new AtomicInteger();

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected TickNative(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "testnatives.tick", parameters);
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
	public static TickNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new TickNative(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		APPLIED.incrementAndGet();
		return state.forEach(arguments[0], (current, value) -> current.returning(value));
	}
}
