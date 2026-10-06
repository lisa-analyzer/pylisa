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
 * Native implementation of {@code bytes.__mul__(self, n)} and
 * {@code bytes.__rmul__(self, n)}, for {@code int} {@code n}: {@code b * n} and
 * {@code n * b} repeat {@code b} {@code n} times (no times, if {@code n} is not
 * positive).
 */
public class BytesMul extends PyNative {

	protected BytesMul(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__mul__", params);
	}

	public static BytesMul build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesMul(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new BinaryExpression(PyBytesType.INSTANCE, args[0], args[1], BytesOperation.REPEAT, getLocation()));
	}
}
