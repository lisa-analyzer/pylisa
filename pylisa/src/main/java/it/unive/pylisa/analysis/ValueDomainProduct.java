package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.combination.ValueLatticeProduct;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.analysis.value.ValueLattice;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

/**
 * The non-reduced product of two value domains: both domains track every value
 * independently, and the abstract state is the pair of their states. The
 * product is at least as precise as each of its components, since a property
 * established by either domain holds.
 * <p>
 * Each component decides by itself which expressions it can process; the
 * product processes an expression whenever at least one component can.
 * </p>
 *
 * @param <L1> the lattice of the first domain
 * @param <L2> the lattice of the second domain
 */
public class ValueDomainProduct<L1 extends ValueLattice<L1>, L2 extends ValueLattice<L2>>
		implements
		ValueDomain<ValueLatticeProduct<L1, L2>> {

	private final ValueDomain<L1> first;

	private final ValueDomain<L2> second;

	/**
	 * Builds the product.
	 *
	 * @param first  the first domain
	 * @param second the second domain
	 */
	public ValueDomainProduct(
			ValueDomain<L1> first,
			ValueDomain<L2> second) {
		this.first = Objects.requireNonNull(first);
		this.second = Objects.requireNonNull(second);
	}

	@Override
	public boolean canProcess(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		return first.canProcess(expression, pp, oracle) || second.canProcess(expression, pp, oracle);
	}

	@Override
	public ValueLatticeProduct<L1, L2> makeLattice() {
		return new ValueLatticeProduct<>(first.makeLattice(), second.makeLattice());
	}

	@Override
	public ValueLatticeProduct<L1, L2> assign(
			ValueLatticeProduct<L1, L2> state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return new ValueLatticeProduct<>(
				first.assign(state.first, id, expression, pp, oracle),
				second.assign(state.second, id, expression, pp, oracle));
	}

	@Override
	public ValueLatticeProduct<L1, L2> smallStepSemantics(
			ValueLatticeProduct<L1, L2> state,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return new ValueLatticeProduct<>(
				first.smallStepSemantics(state.first, expression, pp, oracle),
				second.smallStepSemantics(state.second, expression, pp, oracle));
	}

	@Override
	public ValueLatticeProduct<L1, L2> assume(
			ValueLatticeProduct<L1, L2> state,
			ValueExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		// the refinements of the components are not used: LiSA's
		// non-relational domains refine summary (weak) locations as if they
		// were single values, and BoundedStringSet refines x != y by removing
		// every string y may be, also when y may be several strings. A
		// component only rules the condition out where it certainly does not
		// hold, which is always sound
		if (first.satisfies(state.first, expression, src, oracle) == Satisfiability.NOT_SATISFIED
				|| second.satisfies(state.second, expression, src, oracle) == Satisfiability.NOT_SATISFIED)
			return new ValueLatticeProduct<>(state.first.bottom(), state.second.bottom());
		return state;
	}

	@Override
	public Satisfiability satisfies(
			ValueLatticeProduct<L1, L2> state,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return meet(first.satisfies(state.first, expression, pp, oracle),
				second.satisfies(state.second, expression, pp, oracle));
	}

	@Override
	public Set<BinaryExpression> constraints(
			ValueDomain<?> requesting,
			ValueLatticeProduct<L1, L2> state,
			ValueExpression e,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Set<BinaryExpression> fromFirst = first.constraints(requesting, state.first, e, pp, oracle);
		Set<BinaryExpression> fromSecond = second.constraints(requesting, state.second, e, pp, oracle);
		if (fromFirst == null || fromSecond == null)
			// one of the components holds bottom for the expression
			return null;
		Set<BinaryExpression> constraints = new HashSet<>(fromFirst);
		constraints.addAll(fromSecond);
		return constraints;
	}

	@Override
	public ValueLatticeProduct<L1, L2> onCallReturn(
			ValueLatticeProduct<L1, L2> entryState,
			ValueLatticeProduct<L1, L2> callres,
			ProgramPoint call)
			throws SemanticException {
		return new ValueLatticeProduct<>(
				first.onCallReturn(entryState.first, callres.first, call),
				second.onCallReturn(entryState.second, callres.second, call));
	}

	/**
	 * Combines what two domains established about the same condition. Both
	 * are over-approximations of the same executions, so a definite answer from
	 * either one holds; two contradicting definite answers mean that no
	 * execution is described.
	 *
	 * @param first  the answer of the first domain
	 * @param second the answer of the second domain
	 *
	 * @return the combined answer
	 */
	private static Satisfiability meet(
			Satisfiability first,
			Satisfiability second) {
		if (first == Satisfiability.BOTTOM || second == Satisfiability.BOTTOM)
			return Satisfiability.BOTTOM;
		if (first == Satisfiability.UNKNOWN)
			return second;
		if (second == Satisfiability.UNKNOWN || first == second)
			return first;
		// the components contradict each other: rather than concluding that
		// no execution gets here, which an imprecise component could cause,
		// nothing is decided
		return Satisfiability.UNKNOWN;
	}
}
