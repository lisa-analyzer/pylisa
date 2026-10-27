package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.EnumerateBuiltin;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.MapBuiltin;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.RangeBuiltin;
import it.unive.pylisa.libraries.ZipBuiltin;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.dicts.DictViews;
import it.unive.pylisa.libraries.itertools.ItertoolsModel;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.libraries.sets.SetModel;
import java.util.Collection;
import java.util.Set;

/**
 * The iteration over an iterable, as done by comprehensions and for loops.
 */
public final class Iteration {

	private Iteration() {
	}

	/**
	 * Yields any of the elements of the iterable (e.g. the one bound to the
	 * variable of a comprehension, or of a for loop): the elements of a list or
	 * of a tuple, the elements of a set (see {@link SetModel}), the keys of a
	 * dictionary, the characters of a {@code str}, the bytes of a
	 * {@code bytes}, the {@code int}s of a range (see {@link RangeBuiltin}),
	 * and the keys, values or items of a view of a dictionary (see
	 * {@link DictViews}). Iterating over values that cannot be iterated (e.g.
	 * an {@code int}) raises {@code TypeError}; the elements of other iterables
	 * (e.g. an object whose class defines {@code __iter__}, that is not called)
	 * are unknown.
	 *
	 * @param analysis   the analysis
	 * @param state      the state where the iterable has been evaluated
	 * @param expression the expression computing the iterable
	 * @param iterable   the iterable
	 * @param pp         the program point iterating over the iterable
	 * @param metas      where the meta variables of the elements are added,
	 *                       that {@code pp} must forget (e.g. the receivers of
	 *                       the tuples of {@code d.items()}, see
	 *                       {@link DictViews})
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
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		return elements(analysis, null, state, expression, iterable, pp, metas);
	}

	/**
	 * Yields any of the elements of the iterable, as
	 * {@link #elements(Analysis, AnalysisState, Expression, SymbolicExpression, Statement, Collection)},
	 * where the elements of a map object are the results of calling its
	 * function (see {@link MapBuiltin#elements}), that are unknown otherwise
	 * (see {@link MapBuiltin#uncalled}).
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the iterable has been evaluated
	 * @param expression      the expression computing the iterable
	 * @param iterable        the iterable
	 * @param pp              the program point iterating over the iterable
	 * @param metas           where the meta variables of the elements are
	 *                            added, that {@code pp} must forget
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			Expression expression,
			SymbolicExpression iterable,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		return elements(interprocedural.getAnalysis(), interprocedural, state, expression, iterable, pp, metas);
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			Expression expression,
			SymbolicExpression iterable,
			Statement pp,
			Collection<Identifier> metas)
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
			else if (Unpacking.isTuple(t))
				result = result.lub(ListModel.elements(analysis, state, iterable, t.asPointerType().getInnerType(),
						pp));
			else if (DictModel.isDict(t))
				result = result.lub(DictModel.keys(analysis, state, iterable, pp));
			else if (t.isStringType())
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(StringType.INSTANCE, loc), pp));
			else if (DictViews.isView(t))
				result = result.lub(DictViews.elements(analysis, state, iterable, t, pp, metas));
			else if (ZipBuiltin.isZip(t))
				result = result.lub(ZipBuiltin.elements(analysis, state, iterable, pp, metas));
			else if (EnumerateBuiltin.isEnumerate(t))
				result = result.lub(EnumerateBuiltin.elements(analysis, state, iterable, pp, metas));
			else if (SetModel.isSet(t))
				result = result.lub(SetModel.elements(analysis, state, iterable, pp));
			else if (ItertoolsModel.isIterator(t))
				result = result.lub(ItertoolsModel.elements(analysis, interprocedural, state, iterable, pp, metas));
			else if (MapBuiltin.isMap(t))
				result = result.lub(interprocedural != null
						? MapBuiltin.elements(interprocedural, state, iterable, pp, metas)
						: MapBuiltin.uncalled(analysis, state, pp));
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
	/**
	 * Whether the given value is surely ({@code SATISFIED}), possibly
	 * ({@code UNKNOWN}) or surely not ({@code NOT_SATISFIED}) iterable: lists,
	 * tuples, dictionaries, their views, {@code str}, {@code bytes}, ranges,
	 * zip, enumerate and map objects, sets, and the iterators of itertools (see
	 * {@link ItertoolsModel}) are, the values of the builtin classes whose
	 * methods are all known (e.g. {@code int}) and functions are not, and the
	 * objects of the other classes might be (e.g. if their class defines
	 * {@code __iter__}).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param value    the value
	 * @param pp       the program point using the value
	 *
	 * @return whether the value is iterable
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability iterable(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			Statement pp)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, pp);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp))
			return Satisfiability.UNKNOWN;
		boolean iterable = false, notIterable = false;
		for (Type t : types)
			if (ListModel.isList(t) || Unpacking.isTuple(t) || DictModel.isDict(t) || t.isStringType()
					|| DictViews.isView(t) || RangeBuiltin.isRange(t) || t instanceof PyBytesType
					|| ZipBuiltin.isZip(t) || EnumerateBuiltin.isEnumerate(t) || MapBuiltin.isMap(t)
					|| SetModel.isSet(t) || ItertoolsModel.isIterator(t))
				iterable = true;
			else if (t.isCodeMemberType()
					|| (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t)))
				notIterable = true;
			else
				return Satisfiability.UNKNOWN;
		if (iterable && notIterable)
			return Satisfiability.UNKNOWN;
		return iterable ? Satisfiability.SATISFIED : Satisfiability.NOT_SATISFIED;
	}

	private static boolean isRange(
			Expression expression) {
		return expression instanceof UnresolvedCall && ((UnresolvedCall) expression).getTargetName().equals("range")
				&& ((UnresolvedCall) expression).getQualifier() == null
				&& !PyNameRef.isAssigned(expression.getCFG().getProgram(), "range");
	}
}
