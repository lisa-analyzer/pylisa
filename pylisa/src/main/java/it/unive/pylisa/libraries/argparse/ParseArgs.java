package it.unive.pylisa.libraries.argparse;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyExceptionType;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;
import java.util.List;

/**
 * The model of {@code argparse.ArgumentParser.parse_args()} on the command
 * line of the program: the call returns the namespace of the parser, whose
 * attributes hold the values the arguments may have, or raises
 * {@code SystemExit}, as it does for {@code --help} and for a command line it
 * rejects. Parsing an explicit list of arguments, or into a given namespace,
 * returns an unknown value. On a parser whose namespace the models do not
 * follow (see {@link ForgetArguments}), the call returns an unknown value and
 * may raise any exception, since a converter or an action may run. The
 * namespace is the same object at every call on the same parser, where Python
 * builds a new one each time.
 */
public class ParseArgs extends LibraryNative {

	private static final int SELF = 0;

	private static final int ARGS = 1;

	private static final int NAMESPACE = 2;

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected ParseArgs(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "ArgumentParser.parse_args", parameters);
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
	public static ParseArgs build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new ParseArgs(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.forEachCombination(List.of(arguments[SELF], arguments[ARGS], arguments[NAMESPACE]),
				(current, values) -> {
					ModelState<A, D> exits = current.raise(PyExceptionType.SYSTEM_EXIT);
					if (!ArgparseModels.isNone(values.get(ARGS)) || !ArgparseModels.isNone(values.get(NAMESPACE)))
						// the parser's converters and actions may run and raise
						return current.returning(new PushAny(Untyped.INSTANCE, callSite())).lub(exits)
								.lub(current.raise(PyExceptionType.BASE_EXCEPTION));
					// the result is a new reference to the object the field
					// points to, as allocations produce references
					SymbolicExpression namespace = new HeapDereference(Untyped.INSTANCE,
							current.field(values.get(SELF), ArgparseModels.NAMESPACE_FIELD), callSite());
					Type type = PyClassType.isRegistered(ArgparseModels.NAMESPACE)
							? new ReferenceType(PyClassType.lookup(ArgparseModels.NAMESPACE))
							: Untyped.INSTANCE;
					// a parser that forgot its namespace gives an unknown value
					return ArgparseModels.onNamespace(current, values.get(SELF),
							(tracked, ns) -> tracked.returning(new HeapReference(type, namespace, callSite())),
							(untracked, held) -> untracked.returning(new PushAny(Untyped.INSTANCE, callSite()))
									.lub(untracked.raise(PyExceptionType.BASE_EXCEPTION)))
							.lub(exits);
				});
	}
}
