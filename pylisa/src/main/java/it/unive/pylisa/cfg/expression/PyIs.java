package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.compare.Is;
import java.util.Set;

/**
 * The identity test {@code left is right} (see
 * {@link PyBinaryDispatch#identical(Type, Type)}). When identity cannot be
 * decided for some pair of runtime types of the operands, the result is the
 * {@link Is} test, whose branches refine the operands (e.g., {@code x} is
 * {@code None} in the true branch of {@code x is None}).
 */
public class PyIs extends BinaryExpression {

	public PyIs(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "is", BoolType.INSTANCE, left, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> ltypes = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtypes = analysis.getRuntimeTypesOf(state, right, this);
		boolean maybeTrue = false, maybeFalse = false;
		if (ltypes.isEmpty() || rtypes.isEmpty() || PyBinaryDispatch.unknownTypes(ltypes, this)
				|| PyBinaryDispatch.unknownTypes(rtypes, this))
			maybeTrue = maybeFalse = true;
		else
			for (Type l : ltypes)
				for (Type r : rtypes) {
					Boolean identical = PyBinaryDispatch.identical(l, r);
					maybeTrue |= identical == null || identical;
					maybeFalse |= identical == null || !identical;
				}

		CodeLocation loc = getLocation();
		if (maybeTrue && maybeFalse)
			// the branches of a condition refine the operands (e.g., x is None
			// in the true branch of x is None)
			return analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.BinaryExpression(BoolType.INSTANCE, left, right, Is.INSTANCE, loc),
					this);
		return analysis.smallStepSemantics(state, new Constant(BoolType.INSTANCE, maybeTrue, loc), this);
	}
}
