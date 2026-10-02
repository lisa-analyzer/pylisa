package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.natives.Expressions;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;
import it.unive.pylisa.libraries.natives.TaggedLocation;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The model of {@code argparse.ArgumentParser.__init__()}: the parser gets a
 * new, empty namespace, allocated at the call. A parser built from parent
 * parsers, with a default for every argument or with prefix characters other
 * than {@code -} is not followed: it forgets its namespace.
 */
public class ParserInit extends LibraryNative {

	private static final int SELF = 0;

	private static final int PARENTS = 5;

	private static final int PREFIX_CHARS = 7;

	private static final int ARGUMENT_DEFAULT = 9;

	private static final int EXIT_ON_ERROR = 13;

	private static final String PARSER = "argparse.ArgumentParser";

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected ParserInit(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "ArgumentParser.__init__", parameters);
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
	public static ParserInit build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new ParserInit(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		Expressions build = new Expressions(callSite());
		return state.forEachCombination(
				List.of(arguments[SELF], arguments[PARENTS], arguments[PREFIX_CHARS], arguments[ARGUMENT_DEFAULT],
						arguments[EXIT_ON_ERROR]),
				(current, values) -> {
					SymbolicExpression parser = values.get(SELF);
					// a subclass may override what parse_args calls, such as
					// error(); without exit_on_error a rejected command line
					// raises ArgumentError instead of exiting
					boolean followed = ArgparseModels.isNone(values.get(1 /* parents */))
							&& ArgparseModels.string(values.get(2 /* prefix_chars */)).filter("-"::equals).isPresent()
							&& ArgparseModels.isNone(values.get(3 /* argument_default */))
							&& ArgparseModels.isTrue(values.get(4 /* exit_on_error */))
							&& exactlyAParser(current, parser);
					if (!followed)
						return ArgparseModels.forget(current, parser, callSite()).returning(build.none());
					Optional<PyClassType> type = PyClassType.isRegistered(ArgparseModels.NAMESPACE)
							? Optional.of(PyClassType.lookup(ArgparseModels.NAMESPACE))
							: Optional.empty();
					if (type.isEmpty())
						return ArgparseModels.forget(current, parser, callSite()).returning(build.none());
					ModelState<A, D> allocated = current.allocate(type.get(),
							new TaggedLocation(callSite(), "namespace"));
					return allocated.forEach(allocated.values(),
							(created, namespace) -> created.write(parser, ArgparseModels.NAMESPACE_FIELD, namespace))
							.returning(build.none());
				});
	}

	/**
	 * Yields whether a parser is certainly an {@code ArgumentParser} and not
	 * an object of a subclass.
	 */
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean exactlyAParser(
			ModelState<A, D> state,
			SymbolicExpression parser)
			throws SemanticException {
		Set<Type> types = state.runtimeTypes(parser);
		if (types.isEmpty())
			return false;
		for (Type type : types)
			if (!(type instanceof ReferenceType reference && reference.getInnerType() instanceof PyClassType of
					&& of.getUnit().getName().equals(PARSER)))
				return false;
		return true;
	}
}
