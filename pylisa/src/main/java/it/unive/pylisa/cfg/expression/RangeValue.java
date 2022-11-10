package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.TernaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.common.Int32Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.operators.SliceCreation;

public class RangeValue extends TernaryExpression {

	public RangeValue(CFG cfg, CodeLocation loc, Expression left, Expression middle, Expression right) {
		super(cfg, loc, "[::]", left, middle, right);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> ternarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression middle,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation loc = getLocation();
		Type type = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
		if (left instanceof Skip)
			left = new Constant(Int32Type.INSTANCE, new RangeBound(0), loc);
		if (middle instanceof Skip)
			middle = new Constant(Int32Type.INSTANCE, new RangeBound(), loc);
		if (right instanceof Skip)
			right = new Constant(Int32Type.INSTANCE, new RangeBound(1), loc);
		return state.smallStepSemantics(
				new it.unive.lisa.symbolic.value.TernaryExpression(
						type, left, middle, right, SliceCreation.INSTANCE, loc),
				this);
	}
}
