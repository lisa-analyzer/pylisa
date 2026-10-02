package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.operators.SliceCreation;

public class RangeValue extends TernaryExpression {

	public RangeValue(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression middle,
			Expression right) {
		super(cfg, loc, "[::]", left, middle, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		return getLeft() + " : " + getMiddle() + " : " + getRight();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdTernarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression middle,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		// TOP fallback (mirrors ListCreation): if the {@code Slice} type was
		// never registered (no library spec brought it in), return the
		// incoming state unchanged rather than crashing the analysis with
		// "The requested type 'Slice' has not been registered before". The
		// slice expression's value is then unmodelled — fine for projects
		// that use slices outside of dataframe code, which is everything
		// pylisa cares about for route discovery.
		if (!PyClassType.isRegistered(LibrarySpecificationProvider.SLICE))
			return state;
		Type type = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
		if (left instanceof Skip)
			left = new Constant(Int32Type.INSTANCE, new RangeBound(0), loc);
		if (middle instanceof Skip)
			middle = new Constant(Int32Type.INSTANCE, new RangeBound(), loc);
		if (right instanceof Skip)
			right = new Constant(Int32Type.INSTANCE, new RangeBound(1), loc);
		return interprocedural.getAnalysis().smallStepSemantics(
				state,
				new it.unive.lisa.symbolic.value.TernaryExpression(
						type, left, middle, right, SliceCreation.INSTANCE, loc),
				this);
	}
}
