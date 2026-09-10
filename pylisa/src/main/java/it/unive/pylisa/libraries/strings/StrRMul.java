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
import it.unive.pylisa.symbolic.operators.StringMult;

/**
 * Native implementation of {@code str.__rmul__(self, other)}: string
 * repetition with the operands swapped ({@code 3 * "x"}). {@code self} is
 * still the string (the caller binds {@code self} to the string operand
 * regardless of which side of {@code *} it appeared on), so this computes
 * the same result as {@link StrMul}.
 */
public class StrRMul extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected StrRMul(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static StrRMul build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrRMul(cfg, location, "__rmul__", exprs[0], exprs[1]);
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
						StringMult.INSTANCE,
						getLocation()),
				st);
	}
}
