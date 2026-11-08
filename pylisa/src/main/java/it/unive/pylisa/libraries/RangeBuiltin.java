package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingSub;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Collection;

/**
 * Native implementation of Python's {@code range(stop)} and
 * {@code range(start, stop[, step])}: a new object of the class {@code Range},
 * whose elements are ints (see {@link #elements}), and whose length is not
 * tracked. The object holds its start, its stop and its step. The arguments
 * must be ints ({@code TypeError} is raised otherwise), and the step must not
 * be zero ({@code ValueError}).
 */
public class RangeBuiltin extends PyNative {

	protected RangeBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "range", params);
	}

	public static RangeBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RangeBuiltin(cfg, location, exprs);
	}

	/**
	 * Whether the values of the given type are ranges.
	 *
	 * @param t the type
	 *
	 * @return whether the values are ranges
	 */
	public static boolean isRange(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.RANGE);
	}

	// the fields of the start, the stop and the step of the ranges
	private static final String[] FIELDS = { "start", "stop", "step" };

	private static AccessChild field(
			SymbolicExpression range,
			String name,
			CodeLocation loc) {
		return new AccessChild(Int32Type.INSTANCE,
				new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.RANGE), range, loc),
				new Variable(Untyped.INSTANCE, name, loc), loc);
	}

	/**
	 * Yields the number of elements of the range, if it is known: when its step
	 * is surely {@code 1} and its stop minus its start is surely a constant (at
	 * most {@code max}), or when it is surely empty.
	 *
	 * @param <A>      the kind of {@link AbstractLattice} of the state
	 * @param <D>      the kind of {@link AbstractDomain} of the analysis
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param range    the range
	 * @param pp       the program point using the range
	 * @param max      the largest length looked for
	 *
	 * @return the number of elements, or {@code null} if it is not known
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Integer length(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression range,
			ProgramPoint pp,
			int max)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AccessChild start = field(range, FIELDS[0], loc), stop = field(range, FIELDS[1], loc),
				step = field(range, FIELDS[2], loc);
		Constant zero = new Constant(Int32Type.INSTANCE, 0, loc);
		if (analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step, zero, ComparisonGt.INSTANCE, loc),
				context, pp) == Satisfiability.SATISFIED
				&& analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, stop, start,
						ComparisonLe.INSTANCE, loc), context, pp) == Satisfiability.SATISFIED)
			// surely empty
			return 0;
		if (analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step,
				new Constant(Int32Type.INSTANCE, 1, loc), ComparisonEq.INSTANCE, loc), context,
				pp) != Satisfiability.SATISFIED)
			return null;
		BinaryExpression size = new BinaryExpression(Int32Type.INSTANCE, stop, start,
				NumericNonOverflowingSub.INSTANCE, loc);
		for (int n = 1; n <= max; n++)
			if (analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, size,
					new Constant(Int32Type.INSTANCE, n, loc), ComparisonEq.INSTANCE, loc), context,
					pp) == Satisfiability.SATISFIED)
				return n;
		return null;
	}

	/**
	 * Yields any of the elements of the range: an int between its start and its
	 * stop, if its step is surely positive or surely negative (and any int
	 * otherwise), held by a fresh variable added to {@code metas}. The state is
	 * bottom if the range is surely empty.
	 *
	 * @param <A>      the kind of {@link AbstractLattice} of the state
	 * @param <D>      the kind of {@link AbstractDomain} of the analysis
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param range    the range
	 * @param pp       the program point iterating over the range
	 * @param metas    where the variable holding the element is added, that
	 *                     {@code pp} must forget
	 *
	 * @return the state whose computed expression is the element
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression range,
			ProgramPoint pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Variable element = new Variable(Int32Type.INSTANCE, "$range@" + loc.getCodeLocation(), loc);
		AnalysisState<A> result = analysis.assign(state, element, new PushAny(Int32Type.INSTANCE, loc), context,
				pp);
		AccessChild start = field(range, FIELDS[0], loc), stop = field(range, FIELDS[1], loc),
				step = field(range, FIELDS[2], loc);
		Constant zero = new Constant(Int32Type.INSTANCE, 0, loc);
		if (analysis.satisfies(result, new BinaryExpression(BoolType.INSTANCE, step, zero, ComparisonGt.INSTANCE,
				loc), context, pp) == Satisfiability.SATISFIED)
			// start <= element < stop
			result = analysis.assume(result, new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, start, element, ComparisonLe.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, element, stop, ComparisonLt.INSTANCE, loc),
					LogicalAnd.INSTANCE, loc), context, pp, pp);
		else if (analysis.satisfies(result, new BinaryExpression(BoolType.INSTANCE, step, zero,
				ComparisonLt.INSTANCE, loc), context, pp) == Satisfiability.SATISFIED)
			// stop < element <= start
			result = analysis.assume(result, new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, stop, element, ComparisonLt.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, element, start, ComparisonLe.INSTANCE, loc),
					LogicalAnd.INSTANCE, loc), context, pp, pp);
		if (result.getExecutionState().isBottom())
			// the range is empty: there is no element
			return state.bottomExecution();
		metas.add(element);
		return result.withExecutionExpression(element);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability typed = hasType(analysis, context, state, args[0], INT);
		for (int i = 1; i < args.length; i++)
			if (!missing(args[i]))
				typed = typed.and(hasType(analysis, context, state, args[i], INT));
		SymbolicExpression step = args[2];
		Satisfiability zero = missing(step) ? Satisfiability.NOT_SATISFIED
				: analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step,
						new Constant(Int32Type.INSTANCE, 0, loc), ComparisonEq.INSTANCE, loc), context, this);

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			if (zero != Satisfiability.SATISFIED) {
				Type range = PyClassType.lookup(LibrarySpecificationProvider.RANGE);
				MemoryAllocation alloc = new MemoryAllocation(range, loc, false);
				HeapReference ref = new HeapReference(new ReferenceType(range), alloc, loc);
				AnalysisState<A> created = Allocations.allocate(analysis, context, state, alloc, st);
				// range(stop) starts from 0, and the step is 1 by default
				boolean single = missing(args[1]);
				SymbolicExpression[] bounds = {
						single ? new Constant(Int32Type.INSTANCE, 0, loc) : args[0],
						single ? args[0] : args[1],
						missing(step) ? new Constant(Int32Type.INSTANCE, 1, loc) : step };
				for (int i = 0; i < FIELDS.length; i++)
					created = analysis.assign(created, field(ref, FIELDS[i], loc), bounds[i], context, st);
				result = result.lub(compute(analysis, context, created, ref));
			}
			if (zero != Satisfiability.NOT_SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
		}
		if (typed != Satisfiability.SATISFIED)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}
}
