package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;

/**
 * Native implementation of Python's {@code range(stop)} and
 * {@code range(start, stop[, step])}: a new object of the class {@code Range},
 * whose elements are ints (see {@code Iteration}), and whose length is not
 * tracked. The arguments must be ints ({@code TypeError} is raised otherwise),
 * and the step must not be zero ({@code ValueError}).
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

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability typed = hasType(analysis, state, args[0], INT);
		for (int i = 1; i < args.length; i++)
			if (!missing(args[i]))
				typed = typed.and(hasType(analysis, state, args[i], INT));
		SymbolicExpression step = args[2];
		Satisfiability zero = missing(step) ? Satisfiability.NOT_SATISFIED
				: analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step,
						new Constant(Int32Type.INSTANCE, 0, loc), ComparisonEq.INSTANCE, loc), this);

		AnalysisState<A> result = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			if (zero != Satisfiability.SATISFIED) {
				Type range = PyClassType.lookup(LibrarySpecificationProvider.RANGE);
				result = result.lub(compute(analysis, state, new HeapReference(new ReferenceType(range),
						new MemoryAllocation(range, loc, false), loc)));
			}
			if (zero != Satisfiability.NOT_SATISFIED)
				result = result.lub(raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
		}
		if (typed != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
