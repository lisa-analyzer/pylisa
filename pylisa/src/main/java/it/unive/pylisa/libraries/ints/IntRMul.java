package it.unive.pylisa.libraries.ints;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
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
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.StringMult;
import java.util.Set;

/**
 * Native implementation of {@code int.__rmul__(self, other)}, i.e. the
 * reflected multiplication {@code other * self}. Multiplication being
 * commutative for numbers, this computes the same result as {@link IntMul}
 * (string-repeat handling included, for the same reason: this codebase's
 * call resolution does not reliably fall back to {@code str.__rmul__} for
 * {@code 3 * "x"}), iterating each of {@code other}'s runtime types
 * individually and joining the results so a merge point (e.g. {@code other}
 * could be either an int or a string) is handled precisely for every
 * contributing type.
 */
public class IntRMul extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected IntRMul(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static IntRMul build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IntRMul(cfg, location, "__rmul__", exprs[0], exprs[1]);
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rtsOther = analysis.getRuntimeTypesOf(state, right, this);

		AnalysisState<A> result = state.bottom();
		for (Type t : rtsOther) {
			if (t.isStringType())
				result = result.lub(analysis.smallStepSemantics(state,
						new it.unive.lisa.symbolic.value.BinaryExpression(
								right.getStaticType(),
								right,
								left,
								StringMult.INSTANCE,
								getLocation()),
						st));
			else if (t.isNumericType())
				result = result.lub(analysis.smallStepSemantics(state,
						new it.unive.lisa.symbolic.value.BinaryExpression(
								getStaticType(),
								left,
								right,
								NumericNonOverflowingMul.INSTANCE,
								getLocation()),
						st));
			// else: this type does not contribute
		}

		return result;
	}
}
