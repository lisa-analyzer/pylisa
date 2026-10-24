package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Python's {@code min} and {@code max}, either on two or more arguments
 * ({@code min(a, b, ...)}) or on the elements of an iterable
 * ({@code min(iterable)}, or {@code min(iterable, default=d)}). The result is
 * one of the values: a value is excluded if another one is surely smaller (or
 * greater, for {@code max}). Values are compared with {@code <}:
 * {@code TypeError} is raised if they cannot be compared (e.g. an {@code int}
 * and a {@code str}), and an empty iterable raises {@code ValueError}, unless a
 * default is given. The comparisons of objects of the classes of the program
 * are not executed: their results, and the errors they might raise, are not
 * known.
 */
public class PyMinMax extends NaryExpression {

	private final boolean max;

	private final int positional;

	private final boolean hasDefault;

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call is
	 * @param location   the location of the call
	 * @param max        whether this is {@code max}
	 * @param positional the positional arguments
	 * @param dflt       the {@code default} argument, or {@code null}
	 */
	public PyMinMax(
			CFG cfg,
			CodeLocation location,
			boolean max,
			Expression[] positional,
			Expression dflt) {
		super(cfg, location, max ? "max" : "min", Untyped.INSTANCE, arguments(positional, dflt));
		this.max = max;
		this.positional = positional.length;
		this.hasDefault = dflt != null;
	}

	private static Expression[] arguments(
			Expression[] positional,
			Expression dflt) {
		if (dflt == null)
			return positional;
		Expression[] args = new Expression[positional.length + 1];
		System.arraycopy(positional, 0, args, 0, positional.length);
		args[positional.length] = dflt;
		return args;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		PyMinMax other = (PyMinMax) o;
		if (max != other.max)
			return Boolean.compare(max, other.max);
		return Boolean.compare(hasDefault, other.hasDefault);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		if (positional == 0 || (hasDefault && positional > 1))
			// e.g. min(), or min(a, b, default=c)
			return raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR);
		return combinations(analysis, state, params, new SymbolicExpression[params.length], 0);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> combinations(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet[] params,
			SymbolicExpression[] args,
			int pos)
			throws SemanticException {
		if (pos == params.length)
			return positional == 1 ? ofIterable(analysis, state, args[0], hasDefault ? args[1] : null)
					: ofValues(analysis, state, List.of(args), true);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression e : params[pos]) {
			args[pos] = e;
			result = result.lub(combinations(analysis, state, params, args, pos + 1));
		}
		return result;
	}

	// min(iterable): the elements of a list or tuple of known length are
	// compared as values (when each has a single value), otherwise the result
	// is any of the elements
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> ofIterable(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterable,
			SymbolicExpression dflt)
			throws SemanticException {
		List<SymbolicExpression> elements = ListModel.exactElements(analysis, state, iterable, this);
		if (elements == null)
			return any(analysis, state, iterable, dflt);
		if (elements.isEmpty())
			return empty(analysis, state, dflt);
		return ofValues(analysis, state, elements, true);
	}

	// min(iterable): any of the elements, that might be empty
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> any(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterable,
			SymbolicExpression dflt)
			throws SemanticException {
		AnalysisState<A> elements = Iteration.elements(analysis, state, getSubExpressions()[0], iterable, this,
				getMetaVariables());
		// e.g. the TypeError raised by iterating an int
		AnalysisState<A> result = elements.bottomExecution();
		if (!elements.getExecutionExpressions().isEmpty())
			result = result.lub(ofValues(analysis, elements, list(elements.getExecutionExpressions()), false));
		return result.lub(empty(analysis, state, dflt));
	}

	private static List<SymbolicExpression> list(
			ExpressionSet expressions) {
		List<SymbolicExpression> list = new ArrayList<>();
		for (SymbolicExpression e : expressions)
			list.add(e);
		return list;
	}

	// the result of an empty iterable
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> empty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dflt)
			throws SemanticException {
		if (dflt != null)
			return analysis.smallStepSemantics(state, dflt, this);
		return raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR);
	}

	/**
	 * The minimum (or maximum) of the values. If {@code exact}, each value is a
	 * single value, compared in order with the current minimum; otherwise, each
	 * value represents any of the elements of an iterable, and any two of them
	 * might be compared.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> ofValues(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			List<SymbolicExpression> values,
			boolean exact)
			throws SemanticException {
		CodeLocation loc = getLocation();
		List<Set<Type>> types = new ArrayList<>();
		for (SymbolicExpression v : values)
			types.add(analysis.getRuntimeTypesOf(state, v, this));

		// python compares each value with the minimum so far: since values
		// compared without errors are comparable to the first one, a value
		// that cannot be compared with the first one raises TypeError
		boolean error = false, mightError = false;
		if (exact)
			for (int i = 1; i < values.size(); i++) {
				Satisfiability sat = comparable(types.get(0), types.get(i));
				error |= sat == Satisfiability.NOT_SATISFIED;
				mightError |= sat == Satisfiability.UNKNOWN;
			}
		else {
			Set<Type> all = new HashSet<>();
			types.forEach(all::addAll);
			// a single element is not compared with anything
			mightError = comparable(all, all) != Satisfiability.SATISFIED;
		}
		if (error)
			return raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR);

		AnalysisState<A> result = state.bottomExecution();
		for (int c = 0; c < values.size(); c++) {
			boolean excluded = false;
			if (exact)
				for (int d = 0; d < values.size() && !excluded; d++)
					if (d != c) {
						// min excludes c if d < c, max if c < d
						SymbolicExpression smaller = max ? values.get(c) : values.get(d);
						SymbolicExpression greater = max ? values.get(d) : values.get(c);
						excluded = analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, smaller,
								greater, ComparisonLt.INSTANCE, loc), this) == Satisfiability.SATISFIED;
					}
			if (!excluded)
				result = result.lub(analysis.smallStepSemantics(state, values.get(c), this));
		}
		if (mightError)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	// whether values of types in left can be compared with values of types
	// in right
	private static Satisfiability comparable(
			Set<Type> left,
			Set<Type> right) {
		if (left.isEmpty() || right.isEmpty())
			return Satisfiability.UNKNOWN;
		boolean maybe = false, maybeNot = false;
		for (Type l : left)
			for (Type r : right) {
				Satisfiability sat = comparable(l, r);
				maybe |= sat != Satisfiability.NOT_SATISFIED;
				maybeNot |= sat != Satisfiability.SATISFIED;
			}
		if (!maybeNot)
			return Satisfiability.SATISFIED;
		return maybe ? Satisfiability.UNKNOWN : Satisfiability.NOT_SATISFIED;
	}

	// whether two values of the given types can be compared with <
	private static Satisfiability comparable(
			Type l,
			Type r) {
		int kl = kind(l), kr = kind(r);
		if (kl == UNKNOWN || kr == UNKNOWN)
			return Satisfiability.UNKNOWN;
		if (kl != kr || kl == UNORDERED)
			// e.g. an int and a str, or None and None
			return Satisfiability.NOT_SATISFIED;
		if (kl == LIST || kl == TUPLE)
			// lists and tuples are compared element by element
			return Satisfiability.UNKNOWN;
		return Satisfiability.SATISFIED;
	}

	private static final int UNKNOWN = 0, NUMBER = 1, STRING = 2, BYTES = 3, LIST = 4, TUPLE = 5, UNORDERED = 6;

	// the kind of the values of the given type, for comparisons: values of
	// different kinds cannot be compared
	private static int kind(
			Type t) {
		if (t.isNumericType() || t.isBooleanType())
			return NUMBER;
		if (t.isStringType())
			return STRING;
		if (t instanceof PyBytesType)
			return BYTES;
		if (t.isNullType() || t.isCodeMemberType() || DictModel.isDict(t))
			return UNORDERED;
		// lists (or tuples) are comparable only to lists (or tuples), element
		// by element
		if (ListModel.isList(t))
			return LIST;
		if (Unpacking.isTuple(t))
			return TUPLE;
		// e.g. an object of a class of the program
		return UNKNOWN;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> raise(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			String exception)
			throws SemanticException {
		return PyExceptions.raise(analysis, state, getCFG(), getLocation(), this, exception);
	}
}
