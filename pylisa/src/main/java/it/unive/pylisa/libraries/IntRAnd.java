package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseAnd;

/**
 * Native implementation of {@code int.__rand__(self, other)}, i.e. the
 * reflected bitwise and {@code other & self}. Bitwise and being commutative,
 * this computes the same result as {@link IntAnd}.
 */
public class IntRAnd extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected IntRAnd(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static IntRAnd build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IntRAnd(cfg, location, "__rand__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
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
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new it.unive.lisa.symbolic.value.BinaryExpression(
						getStaticType(),
						left,
						right,
						BitwiseAnd.INSTANCE,
						getLocation()),
				st);
	}
}
