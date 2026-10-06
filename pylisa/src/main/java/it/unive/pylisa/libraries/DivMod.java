package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.Modulo;

/**
 * Native implementation of {@code int.__divmod__(self, other)} and
 * {@code float.__divmod__(self, other)}: the tuple
 * {@code (self // other, self % other)}, raising {@code ZeroDivisionError} if
 * {@code other} is zero. {@link RDivMod} is the reflected {@code __rdivmod__}.
 */
public class DivMod extends PyNative {

	private final boolean reflected;

	protected DivMod(
			CFG cfg,
			CodeLocation location,
			String name,
			boolean reflected,
			Expression[] params) {
		super(cfg, location, name, params);
		this.reflected = reflected;
	}

	public static DivMod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DivMod(cfg, location, "__divmod__", false, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		// __rdivmod__(self, other) is divmod(other, self)
		SymbolicExpression dividend = reflected ? args[1] : args[0];
		SymbolicExpression divisor = reflected ? args[0] : args[1];

		BinaryExpression zero = new BinaryExpression(BoolType.INSTANCE, divisor,
				new Constant(Int32Type.INSTANCE, 0, loc), ComparisonEq.INSTANCE, loc);
		Satisfiability byZero = analysis.satisfies(state, zero, this);

		AnalysisState<A> result = state.bottom();
		if (byZero != Satisfiability.SATISFIED) {
			// the same quotient and remainder of // and %
			BinaryExpression quotient = new BinaryExpression(Untyped.INSTANCE, dividend, divisor,
					FloorDivision.INSTANCE, loc);
			BinaryExpression remainder = new BinaryExpression(Untyped.INSTANCE, dividend, divisor,
					Modulo.INSTANCE, loc);
			result = result.lub(TupleCreation.create(analysis, state, getOriginatingStatement(), loc,
					new ExpressionSet[] { new ExpressionSet(quotient), new ExpressionSet(remainder) }));
		}
		if (byZero != Satisfiability.NOT_SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.ZERO_DIVISION_ERROR));
		return result;
	}
}
