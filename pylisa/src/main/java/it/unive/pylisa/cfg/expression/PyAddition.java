package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.numeric.Addition;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.symbolic.operators.StringAdd;
import java.util.Set;

public class PyAddition extends Addition {

	/**
	 * Builds the addition.
	 *
	 * @param cfg      the {@link CFG} where this operation lies
	 * @param location the location where this literal is defined
	 * @param left     the left-hand side of this operation
	 * @param right    the right-hand side of this operation
	 */
	public PyAddition(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type> rtsl = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = interprocedural.getAnalysis().getRuntimeTypesOf(state, right, this);

		// STRING ADD (CONCATENATION)
		if (rtsl.stream().anyMatch(Type::isStringType)
				&& rtsr.stream().anyMatch(Type::isStringType)) {
			return interprocedural.getAnalysis().smallStepSemantics(
					state,
					new BinaryExpression(
							getStaticType(),
							left,
							right,
							StringAdd.INSTANCE,
							getLocation()),
					this);
		}
		// TODO: OTHER CASES:
		// - List (The + operator returns a list containing all the elements of
		// the first and the second list (second list appended to first)
		// - Tuple (The + operator works like List, i.e. it returns a Tuple of M
		// = n1+n2 elements where the first n1 elements are all the elements of
		// the first (right) tuple
		// and the last n2 are all the elements of the second (right) tuple.
		// Set and Dict does not support operand +
		return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
	}
}