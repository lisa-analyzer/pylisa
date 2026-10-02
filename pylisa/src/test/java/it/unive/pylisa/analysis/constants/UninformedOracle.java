package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.events.EventQueue;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * A {@link SemanticOracle} with no information about the program: it posts no
 * events, knows no types, performs no heap rewriting and decides no aliasing or
 * reachability question. It lets a value domain be exercised on its own, with
 * results that depend only on the domain's abstract values.
 */
final class UninformedOracle implements SemanticOracle {

	/**
	 * The singleton instance.
	 */
	static final UninformedOracle INSTANCE = new UninformedOracle();

	private UninformedOracle() {
	}

	@Override
	public EventQueue getEventQueue() {
		return null;
	}

	@Override
	public boolean hasWholeValueAnlysis() {
		return false;
	}

	@Override
	public Set<BinaryExpression> constraints(
			ValueDomain<?> requesting,
			ValueExpression e,
			ProgramPoint pp) {
		return Collections.emptySet();
	}

	@Override
	public Set<Type> getRuntimeTypesOf(
			SymbolicExpression e,
			ProgramPoint pp) {
		return Collections.emptySet();
	}

	@Override
	public Type getDynamicTypeOf(
			SymbolicExpression e,
			ProgramPoint pp) {
		return e.getStaticType();
	}

	@Override
	public ExpressionSet rewrite(
			SymbolicExpression expression,
			ProgramPoint pp) {
		return new ExpressionSet(expression);
	}

	@Override
	public ExpressionSet rewrite(
			ExpressionSet expressions,
			ProgramPoint pp) {
		return expressions;
	}

	@Override
	public Satisfiability alias(
			SymbolicExpression x,
			SymbolicExpression y,
			ProgramPoint pp) {
		return Satisfiability.UNKNOWN;
	}

	@Override
	public ExpressionSet reachableFrom(
			SymbolicExpression e,
			ProgramPoint pp) {
		return new ExpressionSet(e);
	}

	@Override
	public Satisfiability isReachableFrom(
			SymbolicExpression x,
			SymbolicExpression y,
			ProgramPoint pp) {
		return Satisfiability.UNKNOWN;
	}

	@Override
	public Satisfiability areMutuallyReachable(
			SymbolicExpression x,
			SymbolicExpression y,
			ProgramPoint pp) {
		return Satisfiability.UNKNOWN;
	}
}
