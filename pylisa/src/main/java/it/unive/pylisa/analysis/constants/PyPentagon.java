package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.symbolic.DefiniteIdSet;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.util.numeric.IntInterval;
import it.unive.lisa.util.numeric.MathNumber;
import it.unive.lisa.util.numeric.MathNumberConversionException;
import it.unive.pylisa.libraries.lists.ListModel;
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

	// whether the expression is a variable that can have relations: an
	// integer holding a single value (see PyValueDomain#single)
	private static boolean tracked(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return expression instanceof Identifier && PyValueDomain.single((Identifier) expression)
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
		return new PyPentagonLattice(assigned, relations.second, relations.copies,
				offsetsAfter(state, id, expression, pp, oracle));
	}

	// the most relations with an offset that a constant assigned to a
	// variable gets with the other variables of known constant value
	private static final int MAX_CONSTANT_RELATIONS = 16;

	// the relations with an offset after id = expression: id = id + c shifts
	// the offsets of id, id = y + c relates id and y (both ways), and id = k
	// relates id to the variables whose value is a known constant (e.g. two
	// counters starting from 0, that are then incremented together);
	// otherwise, id has no relations
	private static OffsetBounds offsetsAfter(
			PyPentagonLattice state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		OffsetBounds offsets = state.offsets;
		if (!named(id) || !PyValueDomain.integral(expression, pp, oracle))
			return offsets.drop(id);
		Linear linear = linear(expression, pp, oracle);
		if (linear != null && linear.id.equals(id))
			return offsets.shift(id, linear.offset);
		offsets = offsets.drop(id);
		if (linear != null)
			return offsets.add(id, linear.id, linear.offset).add(linear.id, id, -linear.offset);
		Integer k = constant(expression);
		if (k == null || state.first.isBottom() || state.first.isTop())
			return offsets;
		int added = 0;
		for (Identifier z : state.first.getKeys()) {
			if (added >= MAX_CONSTANT_RELATIONS)
				break;
			if (z.equals(id) || !named(z))
				continue;
			Long d = singleton(state.first.getState(z).getInterval());
			if (d == null)
				continue;
			offsets = offsets.add(id, z, k - d).add(z, id, d - k);
			added++;
		}
		return offsets;
	}

	// whether the identifier can have relations with an offset: a variable of
	// the program or the length of a list; the other fields of the objects
	// (e.g. the elements of a list, start and step of a range) and the
	// counters of the loops cannot, as their relations change at each
	// iteration and only delay the convergence of the loops (the intervals
	// of the elements are then widened)
	private static boolean named(
			Identifier id) {
		if (id.isWeak() || !PyValueDomain.single(id))
			return false;
		if (id instanceof AllocationSite)
			// the length of a list (e.g. i <= len(l) - 1), not its elements
			return ListModel.LENGTH.equals(((AllocationSite) id).getField());
		return (id instanceof Variable || id instanceof GlobalVariable) && !id.getName().startsWith("__");
	}

	// the value of a singleton interval, or null
	private static Long singleton(
			IntInterval interval) {
		if (interval == null || interval.isBottom() || !interval.isSingleton() || !interval.getLow().isFinite())
			return null;
		try {
			long v = interval.getLow().toLong();
			return v > Integer.MAX_VALUE || v < Integer.MIN_VALUE ? null : v;
		} catch (MathNumberConversionException e) {
			return null;
		}
	}

	// a variable that can have relations, plus an integer constant
	private static final class Linear {
		private final Identifier id;
		private final long offset;

		private Linear(
				Identifier id,
				long offset) {
			this.id = id;
			this.offset = offset;
		}
	}

	// the expression as x + c, for a variable x that can have relations (see
	// tracked) and an integer constant c other than 0 (x, x + c, c + x and
	// x - c), or null: x + 0 is the value of x, that an assumption must not
	// relate to other variables (see RangeBuiltin#bound)
	private static Linear linear(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (tracked(expression, pp, oracle) && named((Identifier) expression))
			return new Linear((Identifier) expression, 0);
		if (!(expression instanceof BinaryExpression))
			return null;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft(), right = (ValueExpression) binary.getRight();
		Integer c = constant(right);
		if (operator instanceof AdditionOperator && tracked(left, pp, oracle) && named((Identifier) left) && c != null
				&& c != 0)
			return new Linear((Identifier) left, c);
		if (operator instanceof SubtractionOperator && tracked(left, pp, oracle) && named((Identifier) left)
				&& c != null && c != 0)
			return new Linear((Identifier) left, -(long) c);
		c = constant(left);
		if (operator instanceof AdditionOperator && tracked(right, pp, oracle) && named((Identifier) right) && c != null
				&& c != 0)
			return new Linear((Identifier) right, c);
		return null;
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
				PyPentagonLattice.environment(state.copies.lattice, equal), state.offsets);
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
				PyPentagonLattice.environment(state.copies.lattice, equal), state.offsets);
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
		result = assumeOffsets(result, expression, src, oracle);
		return reduce(result, src, dest, oracle);
	}

	// the state where the relations with an offset also include the
	// comparisons of the expression between x + a and y + b (see linear)
	private PyPentagonLattice assumeOffsets(
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
			return negated == null ? state : assumeOffsets(state, negated, pp, oracle);
		}
		if (!(expression instanceof BinaryExpression))
			return state;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft(), right = (ValueExpression) binary.getRight();
		if (operator == LogicalAnd.INSTANCE)
			return assumeOffsets(assumeOffsets(state, left, pp, oracle), right, pp, oracle);
		if (operator == LogicalOr.INSTANCE)
			return assumeOffsets(state, left, pp, oracle).lub(assumeOffsets(state, right, pp, oracle));
		Linear l = linear(left, pp, oracle), r = linear(right, pp, oracle);
		if (l == null || r == null || l.id.equals(r.id))
			return state;
		// x + a op y + b, that is x - y op d
		Identifier x = l.id, y = r.id;
		long d = r.offset - l.offset;
		OffsetBounds offsets = state.offsets;
		if (PyValueDomain.lt(operator))
			offsets = offsets.add(x, y, d - 1);
		else if (PyValueDomain.le(operator))
			offsets = offsets.add(x, y, d);
		else if (PyValueDomain.gt(operator))
			offsets = offsets.add(y, x, -d - 1);
		else if (PyValueDomain.ge(operator))
			offsets = offsets.add(y, x, -d);
		else if (operator == ComparisonEq.INSTANCE)
			offsets = offsets.add(x, y, d).add(y, x, -d);
		else
			return state;
		return state.withOffsets(offsets);
	}

	// whether x + a op y + b holds, from the relations with an offset between
	// x and y: x - y <= up and y - x <= down
	private static Satisfiability offsetSatisfies(
			PyPentagonLattice state,
			BinaryOperator operator,
			ValueExpression left,
			ValueExpression right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Linear l = linear(left, pp, oracle), r = linear(right, pp, oracle);
		if (l == null || r == null || l.id.equals(r.id))
			return Satisfiability.UNKNOWN;
		long d = r.offset - l.offset;
		Long up = state.offsets.get(l.id, r.id), down = state.offsets.get(r.id, l.id);
		// x - y is in [low, high]
		Long high = up, low = down == null ? null : -down;
		if (PyValueDomain.lt(operator))
			return high != null && high < d ? Satisfiability.SATISFIED
					: low != null && low >= d ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (PyValueDomain.le(operator))
			return high != null && high <= d ? Satisfiability.SATISFIED
					: low != null && low > d ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (PyValueDomain.gt(operator))
			return low != null && low > d ? Satisfiability.SATISFIED
					: high != null && high <= d ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (PyValueDomain.ge(operator))
			return low != null && low >= d ? Satisfiability.SATISFIED
					: high != null && high < d ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (operator == ComparisonEq.INSTANCE)
			return high != null && low != null && high == d && low == d ? Satisfiability.SATISFIED
					: (high != null && high < d) || (low != null && low > d) ? Satisfiability.NOT_SATISFIED
							: Satisfiability.UNKNOWN;
		if (operator == ComparisonNe.INSTANCE)
			return (high != null && high < d) || (low != null && low > d) ? Satisfiability.SATISFIED
					: high != null && low != null && high == d && low == d ? Satisfiability.NOT_SATISFIED
							: Satisfiability.UNKNOWN;
		return Satisfiability.UNKNOWN;
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
				// a summary of many values (see PyValueDomain#single) is not
				// refined, and it does not refine the other variable either:
				// the relation holds for all its values, also when there are
				// none (e.g. the elements of an empty list)
				if (!PyValueDomain.single(x) || !PyValueDomain.single(y))
					continue;
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
		// x <= y + c: x is at most the upper bound of y plus c, and y at least
		// the lower bound of x minus c
		for (Identifier x : state.offsets.identifiers())
			for (Map.Entry<Identifier, Long> relation : state.offsets.above(x).entrySet()) {
				Identifier y = relation.getKey();
				long c = relation.getValue();
				if (!refined.knowsIdentifier(x) || !refined.knowsIdentifier(y))
					continue;
				Long high = bound(refined.getState(y).getInterval(), true);
				if (high != null && fits(high + c))
					refined = values.assume(refined, new BinaryExpression(BoolType.INSTANCE, x,
							new Constant(Int32Type.INSTANCE, (int) (high + c), src.getLocation()),
							ComparisonLe.INSTANCE, src.getLocation()), src, dest, oracle);
				Long low = bound(refined.getState(x).getInterval(), false);
				if (low != null && fits(low - c))
					refined = values.assume(refined, new BinaryExpression(BoolType.INSTANCE, y,
							new Constant(Int32Type.INSTANCE, (int) (low - c), src.getLocation()),
							ComparisonGe.INSTANCE, src.getLocation()), src, dest, oracle);
				if (refined.isBottom())
					return state.bottom();
			}
		return state.with(refined, state.second);
	}

	// the finite upper (or lower) bound of the interval, or null
	private static Long bound(
			IntInterval interval,
			boolean upper) {
		if (interval == null || interval.isBottom())
			return null;
		MathNumber n = upper ? interval.getHigh() : interval.getLow();
		if (!n.isFinite())
			return null;
		try {
			return n.toLong();
		} catch (MathNumberConversionException e) {
			return null;
		}
	}

	// whether the value fits an int
	private static boolean fits(
			long value) {
		return value <= Integer.MAX_VALUE && value >= Integer.MIN_VALUE;
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
		Satisfiability offset = offsetSatisfies(state, operator, left, right, pp, oracle);
		if (offset != Satisfiability.UNKNOWN)
			return offset;
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
