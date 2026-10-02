package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.natives.ModelState;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Optional;
import java.util.Set;

/**
 * What the models of {@code argparse} share.
 * <p>
 * A parser object keeps, in the field {@link #NAMESPACE_FIELD}, the namespace
 * object that {@code parse_args()} returns. The models of
 * {@code add_argument} write into that namespace, for each argument, the
 * values the attribute of the argument may have after parsing: the parsed
 * command line is unknown, so a value is known only by its type and by the
 * default the argument falls back to. When a call cannot be followed exactly
 * (a flag that is not a string literal, an action that is not modelled, a
 * call of {@code set_defaults}), the field is overwritten with an unknown
 * value, and the parser no longer gives a namespace whose attributes are
 * known.
 * </p>
 */
final class ArgparseModels {

	/**
	 * The qualified name of the class of namespaces.
	 */
	static final String NAMESPACE = "argparse.Namespace";

	/**
	 * The field of a parser that holds the namespace its {@code parse_args()}
	 * returns.
	 */
	static final String NAMESPACE_FIELD = "$argparse.namespace";

	/**
	 * The prefix of the fields of a namespace that record which of its
	 * attributes an argument stored, followed by the name of the attribute.
	 */
	static final String STORED_MARKER = "$argparse.stored:";

	/**
	 * The value the specification gives to the {@code default} parameter of
	 * {@code add_argument} when a call does not pass it, so that the models
	 * can tell an explicit {@code default=None} from no default.
	 */
	static final String DEFAULT_NOT_GIVEN = "$argparse.default-not-given";

	private ArgparseModels() {
	}

	/**
	 * Yields the string a value certainly is.
	 *
	 * @param value the value
	 *
	 * @return the string, or empty if the value is not a string literal
	 */
	static Optional<String> string(
			SymbolicExpression value) {
		return value instanceof Constant constant && !(value instanceof PyNoneConstant)
				&& constant.getValue() instanceof String string ? Optional.of(string) : Optional.empty();
	}

	/**
	 * Yields whether a value is certainly {@code None}.
	 *
	 * @param value the value
	 *
	 * @return {@code true} if it is the {@code None} literal
	 */
	static boolean isNone(
			SymbolicExpression value) {
		return value instanceof PyNoneConstant;
	}

	/**
	 * Yields whether a value is certainly {@code True}.
	 *
	 * @param value the value
	 *
	 * @return {@code true} if it is the {@code True} literal
	 */
	static boolean isTrue(
			SymbolicExpression value) {
		return value instanceof Constant constant && Boolean.TRUE.equals(constant.getValue());
	}

	/**
	 * Makes a parser forget its namespace: from now on its
	 * {@code parse_args()} returns an unknown value.
	 *
	 * @param <A>    the kind of abstract state
	 * @param <D>    the kind of abstract domain
	 * @param state  the state
	 * @param parser the parser
	 * @param site   the location of the call that makes it forget
	 *
	 * @return the state after forgetting
	 *
	 * @throws SemanticException if the field cannot be written
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> forget(
			ModelState<A, D> state,
			SymbolicExpression parser,
			CodeLocation site)
			throws SemanticException {
		return state.write(parser, NAMESPACE_FIELD, new PushAny(Untyped.INSTANCE, site));
	}

	/**
	 * Applies a step to each namespace a parser holds, and leaves the state
	 * unchanged where the parser holds no namespace it tracks.
	 *
	 * @param <A>    the kind of abstract state
	 * @param <D>    the kind of abstract domain
	 * @param state  the state
	 * @param parser the parser
	 * @param step   the step, receiving the namespace
	 *
	 * @return the join of the outcomes
	 *
	 * @throws SemanticException if a step fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> onNamespace(
			ModelState<A, D> state,
			SymbolicExpression parser,
			ModelState.Step<A, D, SymbolicExpression> step)
			throws SemanticException {
		return onNamespace(state, parser, step, (current, namespace) -> current);
	}

	/**
	 * Applies a step to each namespace a parser holds, and another step where
	 * the parser holds no namespace it tracks.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param state     the state
	 * @param parser    the parser
	 * @param step      the step, receiving the namespace
	 * @param untracked the step where the namespace is not tracked, receiving
	 *                      the value the parser holds
	 *
	 * @return the join of the outcomes
	 *
	 * @throws SemanticException if a step fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> onNamespace(
			ModelState<A, D> state,
			SymbolicExpression parser,
			ModelState.Step<A, D, SymbolicExpression> step,
			ModelState.Step<A, D, SymbolicExpression> untracked)
			throws SemanticException {
		ModelState<A, D> read = state.read(parser, NAMESPACE_FIELD);
		return state.forEach(read.values(), (current, namespace) -> {
			Set<Type> types = read.runtimeTypes(namespace);
			boolean tracked = !types.isEmpty() && types.stream().allMatch(Type::isPointerType);
			return tracked ? step.apply(current, namespace) : untracked.apply(current, namespace);
		});
	}
}
