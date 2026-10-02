package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;

/**
 * The model of {@code argparse.ArgumentParser.add_mutually_exclusive_group()}
 * and {@code add_argument_group()}: the call returns the parser itself, so
 * that the arguments added to the group are added to the parser. The
 * constraints of a group (at most one, or exactly one, of its arguments is
 * given) are not modelled, so every combination of the attributes of the
 * arguments is kept.
 */
public class SameParser extends LibraryNative {

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected SameParser(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "ArgumentParser group", parameters);
	}

	/**
	 * Builds the model of one call. This is the factory the library loader
	 * uses for native implementations.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 *
	 * @return the model
	 */
	public static SameParser build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new SameParser(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.forEach(arguments[0], ModelState::returning);
	}
}
