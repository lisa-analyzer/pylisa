package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.pylisa.symbolic.operators.BoolBitwise;

/**
 * Native implementation of {@code bool.__xor__(self, other)} (and of its
 * reflected version) between two {@code bool}s: {@code self ^ other}, a
 * {@code bool}. With an {@code int}, the method of {@code int} applies.
 */
public class BoolXor extends PyNative {

	protected BoolXor(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "^", params);
	}

	public static BoolXor build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BoolXor(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new BinaryExpression(BoolType.INSTANCE, args[0], args[1], BoolBitwise.XOR, getLocation()));
	}
}
