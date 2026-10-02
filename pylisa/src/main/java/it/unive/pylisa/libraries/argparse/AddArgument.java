package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * The model of {@code argparse.ArgumentParser.add_argument()}: the attribute
 * of the argument in the namespace of the parser gets the values it may have
 * after parsing.
 * <p>
 * The attribute is named as {@code argparse} names it: {@code dest} if given,
 * otherwise the name of a positional argument, otherwise the first long flag
 * (or, without one, the first short flag) without its leading dashes and with
 * the other dashes turned into underscores. Its values depend on the action:
 * </p>
 * <ul>
 * <li>{@code store} (the default), with no {@code nargs} and no
 * {@code type}: any string, and also the default ({@code None} unless given)
 * unless the argument is positional or {@code required=True};</li>
 * <li>{@code store_true} and {@code store_false}: any boolean, and also the
 * default if one is given, even {@code None};</li>
 * <li>{@code help} and {@code version}: no attribute;</li>
 * <li>any other action, or {@code nargs} or {@code type} given: an unknown
 * value.</li>
 * </ul>
 * <p>
 * A flag, a {@code dest} or an action that is not a string literal cannot be
 * followed: the parser then forgets its namespace. The call returns an
 * unknown value, standing for the action object {@code argparse} returns.
 * </p>
 */
public class AddArgument extends LibraryNative {

	private static final int SELF = 0;

	private static final int FIRST = 1;

	private static final int THIRD = 3;

	private static final int ACTION = 4;

	private static final int NARGS = 5;

	private static final int DEFAULT = 7;

	private static final int TYPE = 8;

	private static final int REQUIRED = 10;

	private static final int DEST = 13;

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected AddArgument(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "ArgumentParser.add_argument", parameters);
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
	public static AddArgument build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new AddArgument(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		PushAny action = new PushAny(Untyped.INSTANCE, callSite());
		return state.forEachCombination(Arrays.asList(arguments),
				(current, values) -> add(current, values).returning(action));
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> add(
			ModelState<A, D> state,
			List<SymbolicExpression> values)
			throws SemanticException {
		SymbolicExpression parser = values.get(SELF);
		List<String> flags = new ArrayList<>();
		for (SymbolicExpression flag : values.subList(FIRST, THIRD + 1))
			if (!ArgparseModels.isNone(flag)) {
				Optional<String> name = ArgparseModels.string(flag);
				if (name.isEmpty())
					return ArgparseModels.forget(state, parser, callSite());
				flags.add(name.get());
			}
		SymbolicExpression given = values.get(DEST);
		Optional<String> dest = ArgparseModels.isNone(given) ? destination(flags) : ArgparseModels.string(given);
		SymbolicExpression actionValue = values.get(ACTION);
		Optional<String> action = ArgparseModels.isNone(actionValue) ? Optional.of("store")
				: ArgparseModels.string(actionValue);
		// an argument named by its dest alone is positional; it is not followed
		if (flags.isEmpty() || dest.isEmpty() || action.isEmpty())
			return ArgparseModels.forget(state, parser, callSite());
		if (action.get().equals("help") || action.get().equals("version"))
			return state;
		if (!ArgparseModels.isNone(values.get(TYPE)))
			// parse_args calls the converter, which may raise anything
			return ArgparseModels.forget(state, parser, callSite());

		SymbolicExpression defaultValue = values.get(DEFAULT);
		boolean hasDefault = !ArgparseModels.string(defaultValue).filter(ArgparseModels.DEFAULT_NOT_GIVEN::equals)
				.isPresent();
		// without a default, an optional argument that is not on the command
		// line is None
		SymbolicExpression fallback = hasDefault ? defaultValue : new PyNoneConstant(callSite());
		List<SymbolicExpression> attribute = new ArrayList<>();
		switch (action.get()) {
		case "store" -> {
			if (!ArgparseModels.isNone(values.get(NARGS)))
				attribute.add(new PushAny(Untyped.INSTANCE, callSite()));
			else {
				attribute.add(new PushAny(StringType.INSTANCE, callSite()));
				boolean positional = !flags.get(0).startsWith("-");
				if (!positional && !ArgparseModels.isTrue(values.get(REQUIRED)))
					attribute.add(fallback);
			}
		}
		case "store_true", "store_false" -> {
			attribute.add(new PushAny(BoolType.INSTANCE, callSite()));
			if (hasDefault)
				attribute.add(fallback);
		}
		default -> attribute.add(new PushAny(Untyped.INSTANCE, callSite()));
		}
		return ArgparseModels.onNamespace(state, parser,
				(current, namespace) -> store(current, namespace, dest.get(), attribute));
	}

	/**
	 * Stores the values of an attribute in the namespace. When an earlier
	 * argument may have stored the same attribute, its values are kept too, as
	 * Python sets the attribute from whichever of them the command line uses;
	 * a marker field records which attributes were stored.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> store(
			ModelState<A, D> state,
			SymbolicExpression namespace,
			String dest,
			List<SymbolicExpression> attribute)
			throws SemanticException {
		String marker = ArgparseModels.STORED_MARKER + dest;
		ModelState<A, D> marked = state.read(namespace, marker);
		boolean earlier = false;
		for (SymbolicExpression value : marked.values())
			for (Type type : marked.runtimeTypes(value))
				earlier |= type.isBooleanType();
		List<SymbolicExpression> values = new ArrayList<>(attribute);
		ModelState<A, D> start = state;
		if (earlier) {
			ModelState<A, D> old = state.read(namespace, dest);
			values.addAll(old.values().elements());
			start = old;
		}
		ModelState<A, D> written = start.forEach(values, (current, value) -> current.write(namespace, dest, value));
		return written.write(namespace, marker, new Constant(BoolType.INSTANCE, true, callSite()));
	}

	/**
	 * Yields the name of the attribute of an argument without {@code dest}.
	 *
	 * @param flags the names or flags of the argument
	 *
	 * @return the name, or empty if the flags are not valid
	 */
	private static Optional<String> destination(
			List<String> flags) {
		if (flags.isEmpty())
			return Optional.empty();
		if (flags.size() == 1 && !flags.get(0).startsWith("-"))
			return Optional.of(flags.get(0));
		if (flags.stream().anyMatch(flag -> !flag.startsWith("-") || flag.equals("-") || flag.equals("--")))
			return Optional.empty();
		String chosen = flags.stream().filter(flag -> flag.startsWith("--")).findFirst().orElse(flags.get(0));
		return Optional.of(chosen.substring(chosen.startsWith("--") ? 2 : 1).replace('-', '_'));
	}
}
