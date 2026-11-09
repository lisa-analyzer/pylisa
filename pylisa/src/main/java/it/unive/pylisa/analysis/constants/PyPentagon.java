package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.symbolic.DefiniteIdSet;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * The product of the constant propagation of pylisa with intervals (see
 * {@link PyValueDomain}) and of the strict upper bounds of pentagons: for each
 * integer variable {@code x}, the variables {@code y} such that {@code x < y}
 * surely holds, and the variables surely equal to {@code x} (its copies, see
 * {@link PyPentagonLattice}). The relations are introduced by the assumptions
 * comparing two variables (also under conjunctions, disjunctions and negations)
 * and by the assignments {@code x = y}, {@code x = y + c} and {@code x = y - c}
 * (for a constant {@code c}); the bounds hold for all the copies of the
 * variables they relate, and they are transitively closed. They decide the
 * comparisons that the intervals cannot decide, and they refine the intervals
 * of the variables they relate. Only the variables holding integers (or
 * booleans) that are not summaries of many values (as the fields of weak
 * allocation sites) have relations.
 */
public class PyPentagon implements ValueDomain<PyPentagonLattice> {

	private final PyValueDomain values = new PyValueDomain();

	@Override
	public PyPentagonLattice makeLattice() {
		return new PyPentagonLattice(values.makeLattice(),
				new ValueEnvironment<>(new DefiniteIdSet(Collections.emptySet(), true)));
	}

	@Override
	public boolean canProcess(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		return values.canProcess(expression, pp, oracle);
	}

	@Override
	public PyPentagonLattice smallStepSemantics(
			PyPentagonLattice state,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return state.with(values.smallStepSemantics(state.first, expression, pp, oracle), state.second);
	}

	// whether the expression is a variable that can have relations
	private static boolean tracked(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return expression instanceof Identifier && !((Identifier) expression).isWeak()
				&& PyValueDomain.integral(expression, pp, oracle);
	}

	@Override
	public PyPentagonLattice assign(
			PyPentagonLattice state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ValueEnvironment<PyValue> assigned = values.assign(state.first, id, expression, pp, oracle);
		if (assigned.isBottom())
			return state.bottom();
		PyPentagonLattice relations = relations(state, id, expression, pp, oracle);
		return new PyPentagonLattice(assigned, relations.second, relations.copies);
	}

	// the relations after id = expression (the values are not changed)
	private PyPentagonLattice relations(
			PyPentagonLattice state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		PyPentagonLattice fresh = state.without(id::equals);
		if (id.isWeak() || !PyValueDomain.integral(expression, pp, oracle))
			return fresh;

		// id = y: a copy of y
		if (tracked(expression, pp, oracle) && !expression.equals(id))
			return fresh.copy(id, (Identifier) expression);

		// id = y + d, for a constant d of the given sign
		Identifier y = null;
		int sign = 0;
		if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			BinaryOperator operator = binary.getOperator();
			ValueExpression left = (ValueExpression) binary.getLeft(), right = (ValueExpression) binary.getRight();
			if (operator instanceof AdditionOperator && tracked(left, pp, oracle) && constant(right) != null) {
				y = (Identifier) left;
				sign = Integer.signum(constant(right));
			} else if (operator instanceof AdditionOperator && tracked(right, pp, oracle)
					&& constant(left) != null) {
				y = (Identifier) right;
				sign = Integer.signum(constant(left));
			} else if (operator instanceof SubtractionOperator && tracked(left, pp, oracle)
					&& constant(right) != null) {
				y = (Identifier) left;
				sign = -Integer.signum(constant(right));
			}
		}
		if (y == null)
			return fresh;
		if (y.equals(id))
			return increment(state, id, sign);
		if (sign == 0)
			return fresh.copy(id, y);
		// y < id, or id < y
		return sign > 0 ? lessThan(fresh, y, id) : lessThan(fresh, id, y);
	}

	// the relations after id = id + d, for a constant d of the given sign: if
	// d > 0, the variables less than (or equal to) the old value are less than
	// the new one, and if d < 0, the new value is less than the bounds (and the
	// copies) of the old one
	private static PyPentagonLattice increment(
			PyPentagonLattice state,
			Identifier id,
			int sign) {
		if (sign == 0)
			return state;
		Set<Identifier> copies = state.equal(id);
		copies.remove(id);
		Map<Identifier, Set<Identifier>> bounds = state.boundsMap();
		Map<Identifier, Set<Identifier>> equal = state.copiesMap();
		// id has no copies anymore
		equal.remove(id);
		for (Set<Identifier> set : equal.values())
			set.remove(id);
		if (sign > 0) {
			// x < id and x == id become x < id; id < z is not known anymore
			bounds.remove(id);
			for (Identifier copy : copies)
				bounds.computeIfAbsent(copy, k -> new HashSet<>()).add(id);
		} else {
			// id < z and id == z become id < z; x < id is not known anymore
			for (Set<Identifier> set : bounds.values())
				set.remove(id);
			bounds.computeIfAbsent(id, k -> new HashSet<>()).addAll(copies);
		}
		return new PyPentagonLattice(state.first, PyPentagonLattice.environment(state.second.lattice, bounds),
				PyPentagonLattice.environment(state.copies.lattice, equal));
	}

	// the value of an integer constant, or null
	private static Integer constant(
			ValueExpression expression) {
		if (expression instanceof Constant && ((Constant) expression).getValue() instanceof Integer)
			return (Integer) ((Constant) expression).getValue();
		return null;
	}

	// the variables greater than (or equal to, if strict does not hold) one
	// of the given ones
	private static Set<Identifier> above(
			PyPentagonLattice state,
			Set<Identifier> ids,
			boolean strict) {
		Set<Identifier> result = new HashSet<>();
		for (Identifier id : ids) {
			if (!strict)
				result.add(id);
			result.addAll(state.bounds(id));
		}
		return result;
	}

	// the variables less than one of the given ones
	private static Set<Identifier> below(
			Map<Identifier, Set<Identifier>> bounds,
			Set<Identifier> ids) {
		Set<Identifier> result = new HashSet<>();
		for (Map.Entry<Identifier, Set<Identifier>> entry : bounds.entrySet())
			if (!Collections.disjoint(entry.getValue(), ids))
				result.add(entry.getKey());
		return result;
	}

	// the state where x < y also holds, transitively closed: x, its copies and
	// the variables less than them are less than y, its copies and their
	// bounds
	private static PyPentagonLattice lessThan(
			PyPentagonLattice state,
			Identifier x,
			Identifier y) {
		Set<Identifier> greater = above(state, state.equal(y), false);
		Map<Identifier, Set<Identifier>> bounds = state.boundsMap();
		Set<Identifier> lower = state.equal(x);
		lower.addAll(below(bounds, state.equal(x)));
		for (Identifier low : lower)
			bounds.computeIfAbsent(low, k -> new HashSet<>()).addAll(greater);
		return state.with(state.first, PyPentagonLattice.environment(state.second.lattice, bounds));
	}

	// the state where x <= y also holds, transitively closed: x and its copies
	// are less than the bounds of y (and of its copies), and the variables less
	// than x (or its copies) are also less than y and its copies
	private static PyPentagonLattice lessOrEqual(
			PyPentagonLattice state,
			Identifier x,
			Identifier y) {
		Set<Identifier> strictly = above(state, state.equal(y), true);
		Set<Identifier> greater = above(state, state.equal(y), false);
		Map<Identifier, Set<Identifier>> bounds = state.boundsMap();
		Set<Identifier> lower = below(bounds, state.equal(x));
		for (Identifier copy : state.equal(x))
			bounds.computeIfAbsent(copy, k -> new HashSet<>()).addAll(strictly);
		for (Identifier low : lower)
			bounds.computeIfAbsent(low, k -> new HashSet<>()).addAll(greater);
		return state.with(state.first, PyPentagonLattice.environment(state.second.lattice, bounds));
	}

	// the state where x == y also holds: x, y and their copies are copies of
	// each other, and they have the same bounds
	private static PyPentagonLattice equalTo(
			PyPentagonLattice state,
			Identifier x,
			Identifier y) {
		Set<Identifier> all = state.equal(x);
		all.addAll(state.equal(y));
		Map<Identifier, Set<Identifier>> equal = state.copiesMap();
		for (Identifier member : all) {
			Set<Identifier> others = new HashSet<>(all);
			others.remove(member);
			equal.put(member, others);
		}
		Set<Identifier> greater = above(state, all, true);
		Map<Identifier, Set<Identifier>> bounds = state.boundsMap();
		Set<Identifier> lower = below(bounds, all);
		for (Identifier member : all)
			bounds.computeIfAbsent(member, k -> new HashSet<>()).addAll(greater);
		for (Identifier low : lower)
			bounds.computeIfAbsent(low, k -> new HashSet<>()).addAll(all);
		return new PyPentagonLattice(state.first, PyPentagonLattice.environment(state.second.lattice, bounds),
				PyPentagonLattice.environment(state.copies.lattice, equal));
	}

	@Override
	public PyPentagonLattice assume(
			PyPentagonLattice state,
			ValueExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		if (state.isBottom())
			return state;
		ValueEnvironment<PyValue> assumed = values.assume(state.first, expression, src, dest, oracle);
		if (assumed.isBottom())
			return state.bottom();
		PyPentagonLattice result = assumeRelations(state.with(assumed, state.second), expression, src, oracle);
		return reduce(result, src, dest, oracle);
	}

	// the state where the relations also include the comparisons of the
	// expression
	private PyPentagonLattice assumeRelations(
			PyPentagonLattice state,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (state.isBottom())
			return state;
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() != LogicalNegation.INSTANCE && unary.getOperator() != LogicalNot.INSTANCE)
				return state;
			ValueExpression negated = PyValueDomain.negate((ValueExpression) unary.getExpression());
			return negated == null ? state : assumeRelations(state, negated, pp, oracle);
		}
		if (!(expression instanceof BinaryExpression))
			return state;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft(), right = (ValueExpression) binary.getRight();
		if (operator == LogicalAnd.INSTANCE)
			return assumeRelations(assumeRelations(state, left, pp, oracle), right, pp, oracle);
		if (operator == LogicalOr.INSTANCE)
			return assumeRelations(state, left, pp, oracle).lub(assumeRelations(state, right, pp, oracle));
		if (!tracked(left, pp, oracle) || !tracked(right, pp, oracle))
			return state;
		Identifier x = (Identifier) left, y = (Identifier) right;
		if (PyValueDomain.lt(operator))
			return lessThan(state, x, y);
		if (PyValueDomain.gt(operator))
			return lessThan(state, y, x);
		if (PyValueDomain.le(operator))
			return lessOrEqual(state, x, y);
		if (PyValueDomain.ge(operator))
			return lessOrEqual(state, y, x);
		if (operator == ComparisonEq.INSTANCE)
			return equalTo(state, x, y);
		return state;
	}

	// the reduction of the state: it is bottom if a variable is less than
	// itself (or one of its copies), and the intervals of the variables are
	// refined by their relations
	private PyPentagonLattice reduce(
			PyPentagonLattice state,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		if (state.isBottom())
			return state;
		ValueEnvironment<PyValue> refined = state.first;
		for (Map.Entry<Identifier, Set<Identifier>> entry : state.boundsMap().entrySet()) {
			Identifier x = entry.getKey();
			if (!Collections.disjoint(entry.getValue(), state.equal(x)))
				return state.bottom();
			for (Identifier y : entry.getValue()) {
				refined = values.assume(refined, new BinaryExpression(BoolType.INSTANCE, x, y,
						ComparisonLt.INSTANCE, src.getLocation()), src, dest, oracle);
				if (refined.isBottom())
					return state.bottom();
			}
		}
		for (Map.Entry<Identifier, Set<Identifier>> entry : state.copiesMap().entrySet())
			for (Identifier y : entry.getValue()) {
				refined = values.assume(refined, new BinaryExpression(BoolType.INSTANCE, entry.getKey(), y,
						ComparisonEq.INSTANCE, src.getLocation()), src, dest, oracle);
				if (refined.isBottom())
					return state.bottom();
			}
		return state.with(refined, state.second);
	}

	@Override
	public Satisfiability satisfies(
			PyPentagonLattice state,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (state.isBottom())
			return Satisfiability.BOTTOM;
		Satisfiability sat = values.satisfies(state.first, expression, pp, oracle);
		if (sat != Satisfiability.UNKNOWN)
			return sat;
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() != LogicalNegation.INSTANCE && unary.getOperator() != LogicalNot.INSTANCE)
				return sat;
			ValueExpression negated = PyValueDomain.negate((ValueExpression) unary.getExpression());
			return negated == null ? sat : satisfies(state, negated, pp, oracle);
		}
		if (!(expression instanceof BinaryExpression))
			return sat;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft(), right = (ValueExpression) binary.getRight();
		if (operator == LogicalAnd.INSTANCE)
			return satisfies(state, left, pp, oracle).and(satisfies(state, right, pp, oracle));
		if (operator == LogicalOr.INSTANCE)
			return satisfies(state, left, pp, oracle).or(satisfies(state, right, pp, oracle));
		if ((PyValueDomain.lt(operator) || PyValueDomain.le(operator)) && tracked(right, pp, oracle)
				&& left instanceof BinaryExpression
				&& ((BinaryExpression) left).getOperator() instanceof SubtractionOperator) {
			// x - c < y, if c > 0 and x <= y (e.g. xs[len(xs) - 1])
			ValueExpression minuend = (ValueExpression) ((BinaryExpression) left).getLeft();
			Integer c = constant((ValueExpression) ((BinaryExpression) left).getRight());
			if (c != null && c > 0 && tracked(minuend, pp, oracle)) {
				Identifier x = (Identifier) minuend, y = (Identifier) right;
				if (state.same(x, y) || state.below(x, y))
					return Satisfiability.SATISFIED;
			}
		}
		if (!tracked(left, pp, oracle) || !tracked(right, pp, oracle))
			return sat;
		Identifier x = (Identifier) left, y = (Identifier) right;
		boolean xy = state.below(x, y), yx = state.below(y, x), same = state.same(x, y);
		if (PyValueDomain.lt(operator))
			return xy ? Satisfiability.SATISFIED : yx || same ? Satisfiability.NOT_SATISFIED : sat;
		if (PyValueDomain.gt(operator))
			return yx ? Satisfiability.SATISFIED : xy || same ? Satisfiability.NOT_SATISFIED : sat;
		if (PyValueDomain.le(operator))
			return xy || same ? Satisfiability.SATISFIED : yx ? Satisfiability.NOT_SATISFIED : sat;
		if (PyValueDomain.ge(operator))
			return yx || same ? Satisfiability.SATISFIED : xy ? Satisfiability.NOT_SATISFIED : sat;
		if (operator == ComparisonEq.INSTANCE)
			return same ? Satisfiability.SATISFIED : xy || yx ? Satisfiability.NOT_SATISFIED : sat;
		if (operator == ComparisonNe.INSTANCE)
			return same ? Satisfiability.NOT_SATISFIED : xy || yx ? Satisfiability.SATISFIED : sat;
		return sat;
	}
}
