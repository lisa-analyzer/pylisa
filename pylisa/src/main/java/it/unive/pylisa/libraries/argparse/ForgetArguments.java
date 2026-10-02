package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;

/**
 * The model of the methods of {@code argparse.ArgumentParser} that change
 * what parsing does, or parse, in a way the models do not follow
 * ({@code set_defaults()}, {@code add_subparsers()},
 * {@code parse_known_args()}, {@code register()} and the like): the parser
 * forgets its namespace, so that its {@code parse_args()} returns an unknown
 * value, and the call returns an unknown value.
 */
public class ForgetArguments extends LibraryNative {

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected ForgetArguments(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "ArgumentParser call the model does not follow", parameters);
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
	public static ForgetArguments build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new ForgetArguments(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.forEach(arguments[0], (current, parser) -> ArgparseModels.forget(current, parser, callSite())
				.returning(new PushAny(Untyped.INSTANCE, callSite())));
	}
}
