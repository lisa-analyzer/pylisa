package it.unive.pylisa.libraries.ints;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;

/**
 * Native implementation of {@code int.__mul__(self, other)}. {@code other} is
 * declared {@code Untyped} (rather than restricted to {@code Int32Type})
 * because this codebase's call resolution does not reliably fall back to
 * {@code str.__rmul__}/{@code list.__rmul__} for {@code 3 * "x"}/
 * {@code 3 * [1]} otherwise (it never even tries {@code int.__mul__} when
 * {@code other} is declared restrictively, but the reflected fallback then also
 * fails to trigger for reasons not fully understood): so {@code int.__mul__} is
 * always tried first here, and this method itself discriminates on
 * {@code other}'s actual runtime type(s), iterating each one individually and
 * joining the results: only the numeric and string-repeat cases it knows how to
 * handle compute a real value (so a merge point where {@code other} could be
 * either an int or a string is handled precisely for both), and it contributes
 * nothing (not a bogus numeric-multiply result) for anything else (e.g.
 * {@code list}), mirroring {@code TypeError}/{@code NotImplemented} for the
 * cases it does not implement.
 */
public class IntMul extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected IntMul(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static IntMul build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IntMul(cfg, location, "__mul__", exprs[0], exprs[1]);
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
		ScopeId<A> context = interprocedural.getCurrentContext();
		// the other operand is an int (e.g. 3 * "x" is computed by
		// str.__rmul__, 3 * [1] by list.__rmul__)
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new it.unive.lisa.symbolic.value.BinaryExpression(
						getStaticType(),
						left,
						right,
						NumericNonOverflowingMul.INSTANCE,
						getLocation()),
				context, st);
	}
}
