package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.PushInv;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.numeric.IntInterval;
import it.unive.lisa.util.numeric.MathNumber;
import it.unive.lisa.util.numeric.MathNumberConversionException;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import it.unive.pylisa.symbolic.operators.Modulo;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import it.unive.pylisa.symbolic.operators.conversions.BoolToInt;
import java.math.RoundingMode;
import java.util.Set;

/**
 * The reduced product of constant propagation
 * ({@link ConstantPropagationDomain}) and of intervals of integers, as in the
 * {@code ConstantPropagationWithIntervals} domain of JLiSA. The intervals
 * follow the semantics of python's integers: they are unbounded, {@code //} and
 * {@code %} round towards minus infinity (e.g. {@code x % n} is in
 * {@code [0, n - 1]} for a positive {@code n}, for any {@code x}), and
 * {@code False} and {@code True} are {@code 0} and {@code 1}. The interval of a
 * value is computed only when its operands surely are integers (see
 * {@link #integral}), and it is the top interval otherwise: a value whose
 * interval is not the top one is an integer. The two components exchange
 * information: an integer constant is a singleton interval, and a singleton
 * interval of an {@code int} is a constant. The comparisons that constant
 * propagation cannot decide are decided on the intervals, and assuming a
 * comparison refines the intervals of the variables it compares.
 */
public class PyValueDomain implements BaseNonRelationalValueDomain<PyValue> {

	private final ConstantPropagationDomain constants = new ConstantPropagationDomain();

	@Override
	public PyValue top() {
		return PyValue.TOP;
	}

	@Override
	public PyValue bottom() {
		return PyValue.BOTTOM;
	}

	@Override
	public boolean canProcess(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		return constants.canProcess(expression, pp, oracle);
	}

	/**
	 * Yields whether the given expression surely evaluates to an integer (where
	 * a {@code bool} is an integer).
	 *
	 * @param expression the expression
	 * @param pp         the program point where it is evaluated
	 * @param oracle     the oracle
	 *
	 * @return whether the expression is an integer
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static boolean integral(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Set<Type> types = oracle.getRuntimeTypesOf(expression, pp);
		return !types.isEmpty() && types.stream().allMatch(PyValueDomain::isInteger);
	}

	private static boolean isInteger(
			Type type) {
		return type.isBooleanType() || (type.isNumericType() && type.asNumericType().isIntegral());
	}

	// whether the given expression surely evaluates to an int, and not to a
	// bool
	private static boolean isInt(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Set<Type> types = oracle.getRuntimeTypesOf(expression, pp);
		return !types.isEmpty()
				&& types.stream().allMatch(t -> t.isNumericType() && t.asNumericType().isIntegral());
	}

	/**
	 * Builds the value of {@code expression} from the two components: an
	 * integer constant refines the interval to a single value, and a singleton
	 * interval of an {@code int} is a constant. When the components disagree,
	 * both are kept as their join.
	 */
	private PyValue reduce(
			ConstantPropagation constant,
			IntInterval interval,
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (constant.isBottom() || interval.isBottom())
			return PyValue.BOTTOM;
		if (!constant.isTop()) {
			Long value = integerOf(constant);
			if (value == null)
				// e.g. a str: the interval is meaningless
				return new PyValue(constant, IntInterval.TOP);
			IntInterval single = new IntInterval(new MathNumber(value), new MathNumber(value));
			if (interval.includes(single))
				return new PyValue(constant, single);
			// the components disagree: their join is sound
			return new PyValue(ConstantPropagation.TOP, interval.lub(single));
		}
		if (interval.isSingleton() && fitsInt(interval.getLow()) && isInt(expression, pp, oracle))
			try {
				return new PyValue(new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, interval.getLow().toInt(), expression.getCodeLocation())),
						interval);
			} catch (MathNumberConversionException e) {
				// not a finite number
			}
		return new PyValue(constant, interval);
	}

	// whether the number is a 32-bit int, as the ints of constant
	// propagation (MathNumber#toInt truncates the others)
	private static boolean fitsInt(
			MathNumber n) {
		return n.isFinite() && n.geq(new MathNumber(Integer.MIN_VALUE)) && n.leq(new MathNumber(Integer.MAX_VALUE));
	}

	// the integer held by the constant, if it is an int or a bool
	private static Long integerOf(
			ConstantPropagation constant) {
		Object v = constant.getConstant();
		if (v instanceof Boolean)
			return (Boolean) v ? 1L : 0L;
		if (v instanceof Integer || v instanceof Long)
			return ((Number) v).longValue();
		return null;
	}

	@Override
	public PyValue evalConstant(
			Constant constant,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return reduce(constants.evalConstant(constant, pp, oracle), IntInterval.TOP, constant, pp, oracle);
	}

	@Override
	public PyValue evalPushAny(
			PushAny pushAny,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		IntInterval interval = pushAny.getStaticType().isBooleanType()
				? new IntInterval(MathNumber.ZERO, MathNumber.ONE)
				: IntInterval.TOP;
		return new PyValue(constants.evalPushAny(pushAny, pp, oracle), interval);
	}

	@Override
	public PyValue evalPushInv(
			PushInv pushInv,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ConstantPropagation c = constants.evalPushInv(pushInv, pp, oracle);
		return c.isBottom() ? PyValue.BOTTOM : new PyValue(c, IntInterval.TOP);
	}

	@Override
	public PyValue evalSkip(
			Skip skip,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ConstantPropagation c = constants.evalSkip(skip, pp, oracle);
		return c.isBottom() ? PyValue.BOTTOM : new PyValue(c, IntInterval.TOP);
	}

	@Override
	public PyValue evalUnaryExpression(
			UnaryExpression expression,
			PyValue arg,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ConstantPropagation c = constants.evalUnaryExpression(expression, arg.getConstant(), pp, oracle);
		IntInterval interval = IntInterval.TOP;
		if (integral(expression.getExpression(), pp, oracle)) {
			IntInterval i = arg.getInterval();
			if (expression.getOperator() == NumericNegation.INSTANCE)
				interval = new IntInterval(i.getHigh().multiply(MathNumber.MINUS_ONE),
						i.getLow().multiply(MathNumber.MINUS_ONE));
			else if (expression.getOperator() == BoolToInt.INSTANCE)
				interval = i;
		}
		return reduce(c, interval, expression, pp, oracle);
	}

	@Override
	public PyValue evalBinaryExpression(
			BinaryExpression expression,
			PyValue left,
			PyValue right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (left.isBottom() || right.isBottom())
			return PyValue.BOTTOM;
		BinaryOperator operator = expression.getOperator();
		ConstantPropagation c = constants.evalBinaryExpression(expression, left.getConstant(), right.getConstant(),
				pp, oracle);
		if (c.isTop() && comparison(operator)) {
			// a comparison that constant propagation cannot decide might be
			// decided on the intervals
			Satisfiability sat = satisfiesBinaryExpression(expression, left, right, pp, oracle);
			if (sat == Satisfiability.SATISFIED || sat == Satisfiability.NOT_SATISFIED)
				return evalConstant(new Constant(expression.getStaticType(), sat == Satisfiability.SATISFIED,
						expression.getCodeLocation()), pp, oracle);
		}

		IntInterval interval = IntInterval.TOP;
		if (integral(expression.getLeft(), pp, oracle) && integral(expression.getRight(), pp, oracle)) {
			IntInterval l = left.getInterval(), r = right.getInterval();
			if (operator instanceof AdditionOperator)
				interval = l.plus(r);
			else if (operator instanceof SubtractionOperator)
				interval = l.diff(r);
			else if (operator instanceof MultiplicationOperator && l.isFinite() && r.isFinite())
				// an infinite bound might be multiplied by 0
				interval = l.mul(r);
			else if (operator == FloorDivision.INSTANCE)
				interval = floorDiv(l, r);
			else if (operator == Modulo.INSTANCE)
				interval = mod(l, r);
		}
		return reduce(c, interval, expression, pp, oracle);
	}

	@Override
	public PyValue evalTernaryExpression(
			TernaryExpression expression,
			PyValue left,
			PyValue middle,
			PyValue right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (left.isBottom() || middle.isBottom() || right.isBottom())
			return PyValue.BOTTOM;
		return reduce(constants.evalTernaryExpression(expression, left.getConstant(), middle.getConstant(),
				right.getConstant(), pp, oracle), IntInterval.TOP, expression, pp, oracle);
	}

	// x // y, rounding towards minus infinity: the result is computed on the
	// negative and on the positive divisors, as dividing by 0 raises
	// ZeroDivisionError
	private static IntInterval floorDiv(
			IntInterval x,
			IntInterval y)
			throws SemanticException {
		IntInterval result = IntInterval.BOTTOM;
		IntInterval positive = y.glb(new IntInterval(MathNumber.ONE, MathNumber.PLUS_INFINITY));
		if (!positive.isBottom())
			result = result.lub(floorDivPositive(x, positive));
		IntInterval negative = y.glb(new IntInterval(MathNumber.MINUS_INFINITY, MathNumber.MINUS_ONE));
		if (!negative.isBottom())
			// x // y = (-x) // (-y)
			result = result.lub(floorDivPositive(negate(x), negate(negative)));
		return result;
	}

	// x // y, for a divisor y >= 1: floor(x / y) increases with x, and it
	// decreases with y for x >= 0, while it increases with y for x < 0
	private static IntInterval floorDivPositive(
			IntInterval x,
			IntInterval y) {
		MathNumber low = x.getLow().isNegative() ? floor(x.getLow(), y.getLow()) : floor(x.getLow(), y.getHigh());
		MathNumber high = x.getHigh().isNegative() ? floor(x.getHigh(), y.getHigh())
				: floor(x.getHigh(), y.getLow());
		return new IntInterval(low, high);
	}

	// floor(a / b), for b >= 1 (possibly plus infinity)
	private static MathNumber floor(
			MathNumber a,
			MathNumber b) {
		if (a.isInfinite())
			return a;
		if (b.isPlusInfinity())
			// a tends to 0, from below if a is negative
			return a.isNegative() ? MathNumber.MINUS_ONE : MathNumber.ZERO;
		return new MathNumber(a.getNumber().divide(b.getNumber(), 0, RoundingMode.FLOOR));
	}

	// x % y, whose result has the sign of y: it is in [0, y - 1] for a
	// positive y, and in [y + 1, 0] for a negative one (modulo by 0 raises
	// ZeroDivisionError)
	private static IntInterval mod(
			IntInterval x,
			IntInterval y)
			throws SemanticException {
		IntInterval result = IntInterval.BOTTOM;
		IntInterval positive = y.glb(new IntInterval(MathNumber.ONE, MathNumber.PLUS_INFINITY));
		if (!positive.isBottom())
			result = result.lub(modPositive(x, positive));
		IntInterval negative = y.glb(new IntInterval(MathNumber.MINUS_INFINITY, MathNumber.MINUS_ONE));
		if (!negative.isBottom())
			// x % y = -((-x) % (-y))
			result = result.lub(negate(modPositive(negate(x), negate(negative))));
		return result;
	}

	// x % y, for a divisor y >= 1: x itself if 0 <= x < y, otherwise any value
	// in [0, y - 1]
	private static IntInterval modPositive(
			IntInterval x,
			IntInterval y) {
		if (!x.getLow().isNegative() && x.getHigh().lt(y.getLow()))
			return x;
		return new IntInterval(MathNumber.ZERO, y.getHigh().subtract(MathNumber.ONE));
	}

	private static IntInterval negate(
			IntInterval i) {
		return new IntInterval(i.getHigh().multiply(MathNumber.MINUS_ONE), i.getLow().multiply(MathNumber.MINUS_ONE));
	}

	@Override
	public Satisfiability satisfies(
			ValueEnvironment<PyValue> environment,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (!environment.isBottom() && !ConstantPropagationDomain.onlyAccepted(expression, pp, oracle))
			return Satisfiability.UNKNOWN;
		return BaseNonRelationalValueDomain.super.satisfies(environment, expression, pp, oracle);
	}

	@Override
	public Satisfiability satisfiesAbstractValue(
			PyValue value,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Satisfiability sat = constants.satisfiesAbstractValue(value.getConstant(), pp, oracle);
		if (sat != Satisfiability.UNKNOWN || value.getInterval().isTop())
			return sat;
		// an integer (see PyValue) is true if it is not 0
		return truth(value.getInterval());
	}

	private static Satisfiability truth(
			IntInterval interval) {
		if (interval.is(0))
			return Satisfiability.NOT_SATISFIED;
		if (!interval.includes(IntInterval.ZERO))
			return Satisfiability.SATISFIED;
		return Satisfiability.UNKNOWN;
	}

	@Override
	public Satisfiability satisfiesConstant(
			Constant constant,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return constants.satisfiesConstant(constant, pp, oracle);
	}

	@Override
	public Satisfiability satisfiesUnaryExpression(
			UnaryExpression expression,
			PyValue arg,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Satisfiability sat = constants.satisfiesUnaryExpression(expression, arg.getConstant(), pp, oracle);
		if (sat != Satisfiability.UNKNOWN || expression.getOperator() != LogicalNot.INSTANCE
				|| arg.getInterval().isTop())
			return sat;
		return truth(arg.getInterval()).negate();
	}

	@Override
	public Satisfiability satisfiesBinaryExpression(
			BinaryExpression expression,
			PyValue left,
			PyValue right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Satisfiability sat = constants.satisfiesBinaryExpression(expression, left.getConstant(),
				right.getConstant(), pp, oracle);
		if (sat == Satisfiability.SATISFIED || sat == Satisfiability.NOT_SATISFIED
				|| left.isBottom() || right.isBottom() || !comparison(expression.getOperator())
				|| !integral(expression.getLeft(), pp, oracle) || !integral(expression.getRight(), pp, oracle))
			return sat;
		Satisfiability compared = compare(expression.getOperator(), left.getInterval(), right.getInterval());
		if (compared != Satisfiability.UNKNOWN)
			return compared;
		if (expression.getLeft() instanceof Identifier && expression.getLeft().equals(expression.getRight()))
			// an integer compared with itself
			return compare(expression.getOperator(), IntInterval.ZERO, IntInterval.ZERO);
		return sat;
	}

	private static boolean comparison(
			BinaryOperator operator) {
		return lt(operator) || le(operator) || gt(operator) || ge(operator) || operator == ComparisonEq.INSTANCE
				|| operator == ComparisonNe.INSTANCE;
	}

	private static boolean lt(
			BinaryOperator operator) {
		return operator == PyComparisonLt.INSTANCE || operator == ComparisonLt.INSTANCE;
	}

	private static boolean le(
			BinaryOperator operator) {
		return operator == PyComparisonLe.INSTANCE || operator == ComparisonLe.INSTANCE;
	}

	private static boolean gt(
			BinaryOperator operator) {
		return operator == PyComparisonGt.INSTANCE || operator == ComparisonGt.INSTANCE;
	}

	private static boolean ge(
			BinaryOperator operator) {
		return operator == PyComparisonGe.INSTANCE || operator == ComparisonGe.INSTANCE;
	}

	// whether l op r holds for all the values of the intervals, for none, or
	// for some
	private static Satisfiability compare(
			BinaryOperator operator,
			IntInterval l,
			IntInterval r) {
		if (lt(operator))
			return l.getHigh().lt(r.getLow()) ? Satisfiability.SATISFIED
					: l.getLow().geq(r.getHigh()) ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (le(operator))
			return l.getHigh().leq(r.getLow()) ? Satisfiability.SATISFIED
					: l.getLow().gt(r.getHigh()) ? Satisfiability.NOT_SATISFIED : Satisfiability.UNKNOWN;
		if (gt(operator))
			return compare(ComparisonLt.INSTANCE, r, l);
		if (ge(operator))
			return compare(ComparisonLe.INSTANCE, r, l);
		Satisfiability eq;
		if (l.isSingleton() && l.equals(r))
			eq = Satisfiability.SATISFIED;
		else if (!l.intersects(r))
			eq = Satisfiability.NOT_SATISFIED;
		else
			eq = Satisfiability.UNKNOWN;
		return operator == ComparisonEq.INSTANCE ? eq : eq.negate();
	}

	@Override
	public Satisfiability satisfiesTernaryExpression(
			TernaryExpression expression,
			PyValue left,
			PyValue middle,
			PyValue right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return constants.satisfiesTernaryExpression(expression, left.getConstant(), middle.getConstant(),
				right.getConstant(), pp, oracle);
	}

	/**
	 * Assumes that the expression holds: the comparisons between integers
	 * refine the intervals of the variables they compare (also under negations,
	 * conjunctions and disjunctions), and the constants are refined by
	 * {@link ConstantPropagationDomain#assume} (e.g. {@code x is None}).
	 */
	@Override
	public ValueEnvironment<PyValue> assume(
			ValueEnvironment<PyValue> environment,
			ValueExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		if (environment.isBottom())
			return environment;
		Satisfiability sat = satisfies(environment, expression, src, oracle);
		if (sat == Satisfiability.NOT_SATISFIED)
			return environment.bottom();
		if (sat == Satisfiability.SATISFIED)
			return environment;

		ValueEnvironment<PyValue> refined = refine(environment, expression, src, oracle);
		if (refined.isBottom())
			return refined;

		ValueEnvironment<ConstantPropagation> assumed = constants.assume(project(refined), expression, src, dest,
				oracle);
		if (assumed.isBottom())
			return environment.bottom();
		ValueEnvironment<PyValue> result = refined;
		for (Identifier id : assumed.getKeys()) {
			ConstantPropagation c = assumed.getState(id);
			PyValue old = refined.getState(id);
			if (c.equals(old.getConstant()))
				continue;
			PyValue updated = reduce(c, old.getInterval(), id, src, oracle);
			if (updated.isBottom())
				return environment.bottom();
			result = result.putState(id, updated);
		}
		return result;
	}

	// the environment of the constants
	private static ValueEnvironment<ConstantPropagation> project(
			ValueEnvironment<PyValue> environment)
			throws SemanticException {
		ValueEnvironment<ConstantPropagation> result = new ValueEnvironment<>(ConstantPropagation.TOP);
		for (Identifier id : environment.getKeys())
			result = result.putState(id, environment.getState(id).getConstant());
		return result;
	}

	// refines the intervals of the variables compared by the expression,
	// assuming that it holds
	private ValueEnvironment<PyValue> refine(
			ValueEnvironment<PyValue> environment,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (environment.isBottom())
			return environment;
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() != LogicalNegation.INSTANCE && unary.getOperator() != LogicalNot.INSTANCE)
				return environment;
			ValueExpression negated = negate((ValueExpression) unary.getExpression());
			return negated == null ? environment : refine(environment, negated, pp, oracle);
		}
		if (!(expression instanceof BinaryExpression))
			return environment;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft();
		ValueExpression right = (ValueExpression) binary.getRight();
		if (operator == LogicalAnd.INSTANCE)
			return refine(refine(environment, left, pp, oracle), right, pp, oracle);
		if (operator == LogicalOr.INSTANCE)
			return refine(environment, left, pp, oracle).lub(refine(environment, right, pp, oracle));
		if (!comparison(operator) || !integral(left, pp, oracle) || !integral(right, pp, oracle))
			return environment;

		IntInterval l = eval(environment, left, pp, oracle).getInterval();
		IntInterval r = eval(environment, right, pp, oracle).getInterval();
		ValueEnvironment<PyValue> result = environment;
		if (left instanceof Identifier && ((Identifier) left).canBeAssigned())
			result = narrow(result, (Identifier) left, l.glb(bound(operator, l, r)), pp, oracle);
		if (right instanceof Identifier && ((Identifier) right).canBeAssigned())
			result = narrow(result, (Identifier) right, r.glb(bound(swap(operator), r, l)), pp, oracle);
		return result;
	}

	private ValueEnvironment<PyValue> narrow(
			ValueEnvironment<PyValue> environment,
			Identifier id,
			IntInterval interval,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (environment.isBottom())
			return environment;
		if (interval.isBottom())
			return environment.bottom();
		PyValue old = environment.getState(id);
		if (interval.equals(old.getInterval()))
			return environment;
		PyValue updated = reduce(old.getConstant(), interval, id, pp, oracle);
		return updated.isBottom() ? environment.bottom() : environment.putState(id, updated);
	}

	// the values of x such that x op v holds, for some value of v, where x is
	// in the given interval
	private static IntInterval bound(
			BinaryOperator operator,
			IntInterval x,
			IntInterval v) {
		MathNumber low = MathNumber.MINUS_INFINITY, high = MathNumber.PLUS_INFINITY;
		if (lt(operator))
			high = v.getHigh().subtract(MathNumber.ONE);
		else if (le(operator))
			high = v.getHigh();
		else if (gt(operator))
			low = v.getLow().add(MathNumber.ONE);
		else if (ge(operator))
			low = v.getLow();
		else if (operator == ComparisonEq.INSTANCE)
			return v;
		else if (v.isSingleton() && !x.isSingleton()) {
			// x != k excludes k only at the bounds of x
			if (x.getLow().equals(v.getLow()))
				low = v.getLow().add(MathNumber.ONE);
			else if (x.getHigh().equals(v.getLow()))
				high = v.getLow().subtract(MathNumber.ONE);
		}
		if (low.gt(high))
			return IntInterval.BOTTOM;
		return new IntInterval(low, high);
	}

	// the operator such that r op' l holds if and only if l op r holds
	private static BinaryOperator swap(
			BinaryOperator operator) {
		if (lt(operator))
			return ComparisonGt.INSTANCE;
		if (le(operator))
			return ComparisonGe.INSTANCE;
		if (gt(operator))
			return ComparisonLt.INSTANCE;
		if (ge(operator))
			return ComparisonLe.INSTANCE;
		return operator;
	}

	// the negation of the expression, if it is a comparison or a combination
	// of comparisons (on integers, not (x < y) is x >= y)
	private static ValueExpression negate(
			ValueExpression expression) {
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() == LogicalNegation.INSTANCE || unary.getOperator() == LogicalNot.INSTANCE)
				return (ValueExpression) unary.getExpression();
			return null;
		}
		if (!(expression instanceof BinaryExpression))
			return null;
		BinaryExpression binary = (BinaryExpression) expression;
		BinaryOperator operator = binary.getOperator();
		ValueExpression left = (ValueExpression) binary.getLeft();
		ValueExpression right = (ValueExpression) binary.getRight();
		BinaryOperator negated;
		if (operator == LogicalAnd.INSTANCE || operator == LogicalOr.INSTANCE) {
			ValueExpression nl = negate(left), nr = negate(right);
			if (nl == null || nr == null)
				return null;
			return new BinaryExpression(binary.getStaticType(), nl, nr,
					operator == LogicalAnd.INSTANCE ? LogicalOr.INSTANCE : LogicalAnd.INSTANCE,
					binary.getCodeLocation());
		}
		if (lt(operator))
			negated = ComparisonGe.INSTANCE;
		else if (le(operator))
			negated = ComparisonGt.INSTANCE;
		else if (gt(operator))
			negated = ComparisonLe.INSTANCE;
		else if (ge(operator))
			negated = ComparisonLt.INSTANCE;
		else if (operator == ComparisonEq.INSTANCE)
			negated = ComparisonNe.INSTANCE;
		else if (operator == ComparisonNe.INSTANCE)
			negated = ComparisonEq.INSTANCE;
		else
			return null;
		return new BinaryExpression(binary.getStaticType(), left, right, negated, binary.getCodeLocation());
	}
}
