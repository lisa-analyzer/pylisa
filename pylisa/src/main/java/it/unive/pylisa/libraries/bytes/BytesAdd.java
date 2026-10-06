package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.BytesOperation;

/**
 * Native implementation of {@code bytes.__add__(self, other)}, for
 * {@code bytes} {@code other}.
 */
public class BytesAdd extends PyNative {

	protected BytesAdd(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__add__", params);
	}

	public static BytesAdd build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesAdd(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new BinaryExpression(PyBytesType.INSTANCE, args[0], args[1], BytesOperation.CONCAT, getLocation()));
	}
}
