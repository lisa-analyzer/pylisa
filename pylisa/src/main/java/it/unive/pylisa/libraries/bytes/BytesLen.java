package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.BytesLength;

/**
 * Native implementation of {@code bytes.__len__(self)}.
 */
public class BytesLen extends PyNative {

	protected BytesLen(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__len__", params);
	}

	public static BytesLen build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesLen(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new UnaryExpression(Int32Type.INSTANCE, args[0], BytesLength.INSTANCE, getLocation()));
	}
}
