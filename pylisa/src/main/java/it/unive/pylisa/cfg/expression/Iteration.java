package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.RangeBuiltin;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * The iteration over an iterable, as done by comprehensions and for loops.
 */
public final class Iteration {

	private Iteration() {
	}

	/**
	 * Yields any of the elements of the iterable (e.g. the one bound to the
	 * variable of a comprehension, or of a for loop): the elements of a list,
	 * the keys of a dictionary, the characters of a {@code str}, the bytes of a
	 * {@code bytes}, and the {@code int}s of a range (see
	 * {@link RangeBuiltin}). Iterating over values that cannot be iterated
	 * (e.g. an {@code int}) raises {@code TypeError}; the elements of other
	 * iterables (e.g. an object whose class defines {@code __iter__}, that is
	 * not called) are unknown.
	 *
	 * @param analysis   the analysis
	 * @param state      the state where the iterable has been evaluated
	 * @param expression the expression computing the iterable
	 * @param iterable   the iterable
	 * @param pp         the program point iterating over the iterable
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Expression expression,
			SymbolicExpression iterable,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		if (isRange(expression))
			// range(...), whose elements are ints
			return analysis.smallStepSemantics(state, new PushAny(Int32Type.INSTANCE, loc), pp);

		Set<Type> types = analysis.getRuntimeTypesOf(state, iterable, pp);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp))
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		AnalysisState<A> result = state.bottomExecution();
		boolean notIterable = false, unknown = false;
		for (Type t : types)
			if (ListModel.isList(t))
				result = result.lub(ListModel.elements(analysis, state, iterable, pp));
			else if (DictModel.isDict(t))
				result = result.lub(DictModel.keys(analysis, state, iterable, pp));
			else if (t.isStringType())
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(StringType.INSTANCE, loc), pp));
			else if (RangeBuiltin.isRange(t))
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Int32Type.INSTANCE, loc), pp));
			else if (t instanceof PyBytesType)
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Int32Type.INSTANCE, loc), pp));
			else if (t.isCodeMemberType() || (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t)))
				// e.g. an int, or a function
				notIterable = true;
			else
				// e.g. an object whose class defines __iter__
				unknown = true;
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp));
		if (notIterable)
			result = result.lub(PyExceptions.raise(analysis, state, pp.getCFG(), loc, pp,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	// whether the expression is a call to the range builtin
	private static boolean isRange(
			Expression expression) {
		return expression instanceof UnresolvedCall && ((UnresolvedCall) expression).getTargetName().equals("range")
				&& ((UnresolvedCall) expression).getQualifier() == null
				&& !PyBuiltinRef.isAssigned(expression.getCFG().getProgram(), "range");
	}
}
