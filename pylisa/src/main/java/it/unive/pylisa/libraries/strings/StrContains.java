package it.unive.pylisa.libraries.strings;

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
import it.unive.lisa.symbolic.value.operator.binary.StringContains;

/**
 * Native implementation of {@code str.__contains__(self, item)}, i.e.
 * substring containment ({@code item in self}). {@code self} is the
 * haystack, {@code item} the needle, matching the SDK's
 * {@link StringContains} argument order (first argument contains the
 * second).
 */
public class StrContains extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected StrContains(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression item) {
		super(cfg, location, constructName, self, item);
	}

	public static StrContains build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrContains(cfg, location, "__contains__", exprs[0], exprs[1]);
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
						StringContains.INSTANCE,
						getLocation()),
				st);
	}
}
