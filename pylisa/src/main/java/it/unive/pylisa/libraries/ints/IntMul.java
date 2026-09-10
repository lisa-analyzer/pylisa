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
 * Native implementation of {@code int.__mul__(self, other)}. {@code other}
 * is declared {@code Untyped} (rather than restricted to {@code Int32Type})
 * because this codebase's call resolution does not reliably fall back to
 * {@code str.__rmul__}/{@code list.__rmul__} for {@code 3 * "x"}/
 * {@code 3 * [1]} otherwise (it never even tries {@code int.__mul__} when
 * {@code other} is declared restrictively, but the reflected fallback then
 * also fails to trigger for reasons not fully understood): so
 * {@code int.__mul__} is always tried first here, and this method itself
 * discriminates on {@code other}'s actual runtime type(s), iterating each
 * one individually and joining the results: only the numeric and
 * string-repeat cases it knows how to handle compute a real value (so a
 * merge point where {@code other} could be either an int or a string is
 * handled precisely for both), and it contributes nothing (not a bogus
 * numeric-multiply result) for anything else (e.g. {@code list}),
 * mirroring {@code TypeError}/{@code NotImplemented} for the cases it does
 * not implement.
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rtsOther = analysis.getRuntimeTypesOf(state, right, this);

		AnalysisState<A> result = state.bottom();
		for (Type t : rtsOther) {
			if (t.isStringType())
				// 3 * "x": string repeat, string first
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
			// else: this type (e.g. list) does not contribute -- int does not
			// implement this, and unlike the numeric/string cases there is no
			// fallback to compute here
		}

		return result;
	}
}
