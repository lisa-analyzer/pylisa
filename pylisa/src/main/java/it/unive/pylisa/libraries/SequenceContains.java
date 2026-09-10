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
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;

/**
 * Native implementation of {@code Sequence.__contains__(self, item)}
 * (element membership for {@code list}/{@code set}/{@code dict}/
 * {@code tuple}). There is no element-tracking abstract domain for
 * sequences in this codebase (mirroring {@link SequenceLen}), so this is
 * imprecise: it always yields {@code top} rather than the true membership
 * result.
 */
public class SequenceContains extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceContains(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression item) {
		super(cfg, location, constructName, self, item);
	}

	public static SequenceContains build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceContains(cfg, location, "__contains__", exprs[0], exprs[1]);
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
				new PushAny(BoolType.INSTANCE, getLocation()), st);
	}
}
