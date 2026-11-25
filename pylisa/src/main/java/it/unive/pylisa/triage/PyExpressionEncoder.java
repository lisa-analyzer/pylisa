package it.unive.pylisa.triage;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseAnd;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseOr;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseXor;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseShiftLeft;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseShiftRight;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.ternary.StringReplace;
import it.unive.lisa.symbolic.value.operator.unary.NumericAbs;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.triage.encoding.ExpressionEncoder;
import it.unive.lisa.triage.encoding.ExpressionEncoderFactory;
import it.unive.lisa.triage.encoding.SortPolicy;
import it.unive.lisa.triage.encoding.SymbolTable;
import it.unive.lisa.triage.encoding.ValueSort;
import it.unive.lisa.triage.smt.FloatingPoints;
import it.unive.lisa.triage.smt.Sort;
import it.unive.lisa.triage.smt.Term;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeTokenType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.PyFloatConstant;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.FloatPower;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import it.unive.pylisa.symbolic.operators.Modulo;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.TrueDivision;
import it.unive.pylisa.symbolic.operators.compare.Is;
import it.unive.pylisa.symbolic.operators.compare.IsInstance;
import it.unive.pylisa.symbolic.operators.compare.IsNot;
import it.unive.pylisa.symbolic.operators.compare.NotIsInstance;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import it.unive.pylisa.symbolic.operators.conversions.BoolToInt;
import it.unive.pylisa.symbolic.operators.conversions.Floor;
import it.unive.pylisa.symbolic.operators.conversions.ToFloat;
import it.unive.pylisa.symbolic.operators.conversions.ToInt;
import it.unive.pylisa.symbolic.operators.conversions.ToRepr;
import it.unive.pylisa.symbolic.operators.strings.StrGetItem;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * An {@link ExpressionEncoder} that encodes the values and the operators of
 * Python, so that the counterexamples relying on them can be confirmed, and the
 * refutations relying on them are sound with respect to Python's semantics:
 * <ul>
 * <li>{@code None} is the null value of {@link ValueSort};</li>
 * <li>integers are mathematical integers, as Python's ones (the operators of
 * LiSA over them are the non-overflowing ones); bitwise operators are exact
 * on integers within 128 bits, and shifts on constant amounts;</li>
 * <li>floating point numbers are 64-bits IEEE 754 numbers, as Python's ones
 * (see {@link #FACTORY});</li>
 * <li>the arithmetic operators of Python ({@code /}, {@code //}, {@code %},
 * {@code **}) follow Python's semantics (e.g., {@code /} always yields a
 * {@code float}, {@code //} rounds toward negative infinity, and the result of
 * {@code %} takes the sign of the divisor), also over operands that might be
 * both {@code int}s and {@code float}s, with the operations over {@code float}s
 * that the theory of floating point numbers cannot express computed through
 * {@link SymbolTable#evaluable(String, Sort, List, Function, SymbolicExpression)};</li>
 * <li>comparisons compare numbers by their value and strings lexicographically,
 * and equality follows Python's rules across types (e.g., {@code True == 1},
 * while strings are only equal to strings);</li>
 * <li>identity is exact for {@code None} and for objects (references);</li>
 * <li>the truthiness of {@code not x}, the conversions {@code int(x)},
 * {@code float(x)}, {@code math.floor(x)}, {@code str(x)} and {@code repr(x)},
 * the indexing and the repetition of strings, and {@code isinstance} with
 * builtin classes are encoded for the values they can be precisely encoded
 * for.</li>
 * </ul>
 * Whatever is not encoded precisely is never precise: an operation on values
 * of unknown kind, or not encoded for some of them (e.g., the conversion of a
 * {@code str} to a {@code float}), yields an unknown, but deterministic,
 * function of the operands, while an operation over operands that cannot be
 * encoded at all (e.g., arithmetic on a string sort), or over floats when they
 * are not encoded, yields a fresh unknown value.
 */
public class PyExpressionEncoder extends ExpressionEncoder {

	/**
	 * The factory of the encoders of Python expressions. All the floating point
	 * numbers of Python are 64-bits ones, so they are encoded as such
	 * ({@link SortPolicy#withDoubleFloats()}).
	 */
	// FIXME the frontend models Python's floats with 32-bits types: it should
	// use 64-bits ones, so that the analysis agrees with the encoding (float
	// literals keep their 64-bits value apart, see PyFloatConstant)
	public static final ExpressionEncoderFactory FACTORY = new ExpressionEncoderFactory() {

		@Override
		public ExpressionEncoder create(
				SortPolicy sorts,
				SymbolTable symbols) {
			return new PyExpressionEncoder(sorts, symbols);
		}

		@Override
		public SortPolicy sorts(
				SortPolicy sorts) {
			return sorts.withDoubleFloats();
		}
	};

	/**
	 * The largest magnitude of the integers that are exactly represented as
	 * 64-bits floating point numbers (2^53).
	 */
	private static final BigInteger EXACT_FLOATS = BigInteger.ONE.shiftLeft(53);

	/**
	 * The integral type used to convert floating point numbers to integers,
	 * whose range is the one where the conversions are precise.
	 */
	private static final Type LONG = Int64Type.INSTANCE;

	private static final BigInteger LONG_BOUND = BigInteger.ONE.shiftLeft(63);

	/**
	 * Builds the encoder.
	 *
	 * @param sorts   the policy deciding the sorts of identifiers
	 * @param symbols the provider of the constants of the encoding
	 */
	public PyExpressionEncoder(
			SortPolicy sorts,
			SymbolTable symbols) {
		super(sorts, symbols);
	}

	@Override
	protected Term encodeConstant(
			Constant expression)
			throws SemanticException {
		if (expression instanceof PyNoneConstant)
			return ValueSort.nullValue();
		Object value = expression.getValue();
		if (expression instanceof PyFloatConstant)
			// a float literal, whose 64-bits value is kept apart from the
			// 32-bits one used by the analyses
			return getSorts().encodesFloats() ? FloatingPoints.literal(((PyFloatConstant) expression).getExact())
					: opaque(expression);
		if (value instanceof Float)
			// other floats are modeled as 32-bits numbers, built from short
			// literals: the shortest decimal representation of the 32-bits
			// number is the one of the literal (for literals with at most 7
			// significant digits), and Python's value is the 64-bits number
			// closest to it
			return getSorts().encodesFloats()
					? FloatingPoints.literal(Double.parseDouble(Float.toString((Float) value)))
					: opaque(expression);
		return super.encodeConstant(expression);
	}

	@Override
	protected Term encodeUnary(
			UnaryExpression expression,
			Term arg)
			throws SemanticException {
		UnaryOperator op = expression.getOperator();
		if (op instanceof LogicalNot)
			return Term.not(truthiness(arg, expression));
		if (op instanceof BoolToInt)
			return boolToInt(arg);
		if (op instanceof ToFloat)
			return toPyFloat(arg, expression);
		if (op instanceof Floor)
			return floor(arg, expression);
		if (op instanceof StringConstructor)
			return str(arg, false, expression);
		if (op instanceof ToRepr)
			return str(arg, true, expression);
		if (op instanceof NumericNegation || op instanceof NumericAbs)
			return negationOrAbs(arg, op instanceof NumericNegation, expression);
		return super.encodeUnary(expression, arg);
	}

	/**
	 * Python's {@code -x} or {@code abs(x)}: over ints (or bools, read as
	 * ints) it is the operation over integers, and over floats the one over
	 * floating point numbers. The operand is required to be a number. Over
	 * other values, the result is an unknown function of the operand.
	 */
	private Term negationOrAbs(
			Term arg,
			boolean negation,
			UnaryExpression expression) {
		String function = negation ? "neg" : "abs";
		java.util.function.UnaryOperator<Term> ints = negation ? PyExpressionEncoder::neg
				: i -> Term.ite(lt(i, Term.integer(0)), neg(i), i);
		String fp = negation ? "fp.neg" : "fp.abs";
		Sort sort = arg.getSort();
		if (sort.isFloatingPoint())
			return Term.app(fp, sort, arg);
		if (sort.equals(Sort.INT) || sort.equals(Sort.BOOL))
			return ints.apply(asInteger(arg));
		if (!isValue(arg))
			return havoc(expression);
		requireNumeric(arg);
		Cases cases = new Cases().add(isInteger(arg), ValueSort.wrap(ints.apply(asInteger(arg))));
		Term floats = isFloat(arg);
		cases.add(floats, ValueSort.wrap(Term.app(fp, Sort.FLOAT64, asFloatIf(floats, arg, expression))));
		return cases.otherwise(function, ValueSort.SORT, List.of(arg), expression);
	}

	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * Python's {@code str.replace} without a count replaces all the occurrences
	 * of the pattern (SMT's {@code str.replace_all}), while an empty pattern
	 * inserts the replacement between the characters: that case yields an
	 * unknown function of the operands.
	 */
	@Override
	protected Term encodeTernary(
			TernaryExpression expression,
			Term left,
			Term middle,
			Term right)
			throws SemanticException {
		if (!(expression.getOperator() instanceof StringReplace))
			return super.encodeTernary(expression, left, middle, right);
		Term s = asString(left, expression);
		Term pattern = asString(middle, expression);
		Term replacement = asString(right, expression);
		return new Cases()
				.add(Term.not(Term.eq(pattern, Term.string(""))),
						Term.app("str.replace_all", Sort.STRING, s, pattern, replacement))
				.otherwise("str.replace", Sort.STRING, List.of(s, pattern, replacement), expression);
	}

	@Override
	protected Term encodeBinary(
			BinaryExpression expression,
			Term left,
			Term right)
			throws SemanticException {
		BinaryOperator op = expression.getOperator();
		if (op instanceof TrueDivision)
			return trueDivision(left, right, expression);
		if (op instanceof FloorDivision)
			return numeric("floordiv", expression, left, right, PyExpressionEncoder::floorDiv,
					(
							l,
							r) -> getSymbols().evaluable("$py.floordiv", Sort.FLOAT64, List.of(l, r),
									args -> floorDiv(args.get(0).doubleValue(), args.get(1).doubleValue()),
									expression));
		if (op instanceof Modulo)
			return numeric("mod", expression, left, right, PyExpressionEncoder::mod,
					(
							l,
							r) -> getSymbols().evaluable("$py.mod", Sort.FLOAT64, List.of(l, r),
									args -> mod(args.get(0).doubleValue(), args.get(1).doubleValue()), expression));
		if (op instanceof FloatPower)
			return floatPower(left, right, expression);
		if (op instanceof Power)
			return power(left, right, expression);
		if ((op instanceof AdditionOperator || op instanceof SubtractionOperator
				|| op instanceof MultiplicationOperator)
				&& (isValue(left) || isValue(right) || left.getSort().equals(Sort.BOOL)
						|| right.getSort().equals(Sort.BOOL))) {
			// values might be ints, bools or floats: the operation is the one
			// of their kinds, with bools promoted to ints and ints promoted to
			// floats if needed
			String smt = op instanceof AdditionOperator ? "+" : op instanceof SubtractionOperator ? "-" : "*";
			String fp = op instanceof AdditionOperator ? "fp.add"
					: op instanceof SubtractionOperator ? "fp.sub" : "fp.mul";
			String function = op instanceof AdditionOperator ? "add"
					: op instanceof SubtractionOperator ? "sub" : "mul";
			return numeric(function, expression, left, right, (
					l,
					r) -> Term.app(smt, Sort.INT, l, r),
					(
							l,
							r) -> Term.app(fp, Sort.FLOAT64, rne(), l, r));
		}
		if (op instanceof ComparisonEq)
			return equality(left, right, expression);
		if (op instanceof ComparisonNe)
			return Term.not(equality(left, right, expression));
		if (op instanceof PyComparisonLt)
			return compare("<", left, right, expression);
		if (op instanceof PyComparisonLe)
			return compare("<=", left, right, expression);
		if (op instanceof PyComparisonGt)
			return compare("<", right, left, expression);
		if (op instanceof PyComparisonGe)
			return compare("<=", right, left, expression);
		if (op instanceof Is)
			return identity(left, right, expression);
		if (op instanceof IsNot)
			return Term.not(identity(left, right, expression));
		if (op instanceof IsInstance)
			return isInstance(left, right, expression);
		if (op instanceof BitwiseAnd || op instanceof BitwiseOr || op instanceof BitwiseXor
				|| op instanceof BitwiseShiftLeft || op instanceof BitwiseShiftRight)
			return bitwise(left, right, expression);
		if (op instanceof NotIsInstance)
			return Term.not(isInstance(left, right, expression));
		if (op instanceof ToInt)
			return toPyInt(left, right, expression);
		if (op instanceof StrGetItem)
			return strGetItem(left, right, expression);
		if (op instanceof StringMult)
			return strMult(left, right, expression);
		return super.encodeBinary(expression, left, right);
	}

	// helpers on the kinds of values

	private static boolean isValue(
			Term t) {
		return t.getSort().equals(ValueSort.SORT);
	}

	/**
	 * Whether the term is of the given sort: a condition on values, and a
	 * constant for terms of other sorts.
	 */
	private static Term is(
			Term t,
			Sort sort) {
		if (isValue(t))
			return ValueSort.is(t, sort);
		return t.getSort().equals(sort) ? Term.TRUE : Term.FALSE;
	}

	/**
	 * The term read as a term of the given sort, that is meaningful only if it
	 * is of that sort (see {@link #is(Term, Sort)}).
	 */
	private static Term as(
			Term t,
			Sort sort) {
		if (t.getSort().equals(sort))
			return t;
		if (isValue(t))
			return ValueSort.unwrap(t, sort);
		return null;
	}

	private Term isFloat(
			Term t) {
		if (!getSorts().encodesFloats())
			return Term.FALSE;
		return Term.or(is(t, Sort.FLOAT64), is(t, Sort.FLOAT32));
	}

	/**
	 * The term, that might be an int (or a bool, read as an int), as an
	 * integer: meaningful only if it is one of them.
	 */
	private static Term asInteger(
			Term t) {
		if (t.getSort().equals(Sort.INT))
			return t;
		if (t.getSort().equals(Sort.BOOL))
			return Term.ite(t, Term.integer(1), Term.integer(0));
		if (isValue(t))
			return Term.ite(ValueSort.is(t, Sort.BOOL),
					Term.ite(ValueSort.unwrap(t, Sort.BOOL), Term.integer(1), Term.integer(0)),
					ValueSort.unwrap(t, Sort.INT));
		return Term.integer(0);
	}

	private static Term isInteger(
			Term t) {
		return Term.or(is(t, Sort.INT), is(t, Sort.BOOL));
	}

	/**
	 * Whether the term is a number (an int, a bool, read as an int, or a
	 * float).
	 */
	private Term isNumber(
			Term t) {
		return Term.or(isInteger(t), isFloat(t));
	}

	/**
	 * Whether the operands of a numeric operation can be numbers, requiring
	 * them to be (see {@link #requireNumeric(Term)}): if they cannot (e.g.,
	 * one is a string), the operation is not encoded.
	 */
	private boolean numericOperands(
			Term left,
			Term right) {
		for (Term t : List.of(left, right))
			if (!isValue(t) && !t.getSort().equals(Sort.INT) && !t.getSort().equals(Sort.BOOL)
					&& !t.getSort().isFloatingPoint())
				return false;
		requireNumeric(left);
		requireNumeric(right);
		return true;
	}

	/**
	 * The term, that might be a number, as a 64-bits floating point number:
	 * integers are converted as Python does (rounding to nearest), precisely if
	 * their magnitude is below 2^63. Meaningful only if the term is a number.
	 */
	private Term asFloat(
			Term t,
			SymbolicExpression origin) {
		if (t.getSort().equals(Sort.FLOAT64))
			return t;
		if (t.getSort().equals(Sort.FLOAT32))
			return toFloat(t, Sort.FLOAT64, origin);
		if (t.getSort().equals(Sort.INT) || t.getSort().equals(Sort.BOOL))
			return intToFloat(asInteger(t), Sort.FLOAT64, 64, origin);
		if (isValue(t)) {
			// the integer is converted only when the value is one (zero
			// otherwise, so that the conversion of other values is not
			// considered imprecise)
			Term integer = Term.ite(isInteger(t), asInteger(t), Term.integer(0));
			Term converted = intToFloat(integer, Sort.FLOAT64, 64, origin);
			return Term.ite(ValueSort.is(t, Sort.FLOAT64), ValueSort.unwrap(t, Sort.FLOAT64),
					Term.ite(ValueSort.is(t, Sort.FLOAT32),
							toFloat(ValueSort.unwrap(t, Sort.FLOAT32), Sort.FLOAT64, origin), converted));
		}
		return getSymbols().fresh(Sort.FLOAT64, origin);
	}

	/**
	 * The term as a float (see {@link #asFloat(Term, SymbolicExpression)}) if
	 * the condition holds, and 1.0 otherwise: the arguments of functions over
	 * floats that are not used when the condition does not hold are fixed, so
	 * that they agree with the functions they stand for (see
	 * {@link SymbolTable#evaluable(String, Sort, List, Function, SymbolicExpression)}).
	 */
	private Term asFloatIf(
			Term condition,
			Term t,
			SymbolicExpression origin) {
		if (condition.equals(Term.TRUE))
			return asFloat(t, origin);
		return Term.ite(condition, asFloat(t, origin), FloatingPoints.literal(1.0));
	}

	/**
	 * An unknown, but deterministic, function of the given arguments, used when
	 * the guard does not hold (never precise then). The result for references
	 * (e.g., the truthiness of a list) depends on the contents of the objects
	 * they point to, that might change while the references do not: when an
	 * argument is a reference, the result is a fresh value instead, unless
	 * the function depends only on the class of the objects (e.g.,
	 * {@code isinstance}, see {@link #CLASS_FUNCTIONS}).
	 */
	private Term unknown(
			String function,
			Sort sort,
			List<Term> arguments,
			SymbolicExpression origin,
			Term guard) {
		List<Term> args = new ArrayList<>(arguments.size());
		List<Term> references = new ArrayList<>();
		for (Term a : arguments) {
			Term arg = isValue(a) || a.getSort().isFloatingPoint() || a.getSort().equals(Sort.INT)
					|| a.getSort().equals(Sort.BOOL) || a.getSort().equals(Sort.STRING) ? a : ValueSort.wrap(a);
			args.add(arg);
			if (isValue(arg))
				references.add(ValueSort.isReference(arg));
		}
		Term applied = getSymbols().apply("$py." + function, sort, args, origin, guard);
		if (references.isEmpty() || CLASS_FUNCTIONS.stream().anyMatch(function::startsWith))
			return applied;
		Term anyReference = references.size() == 1 ? references.get(0)
				: Term.or(references.toArray(Term[]::new));
		return Term.ite(anyReference, getSymbols().fresh(sort, origin, guard), applied);
	}

	/**
	 * The prefixes of the names of the unknown functions whose results depend
	 * only on the classes of the objects their arguments point to, that never
	 * change: they are functions also of references (see
	 * {@link #unknown(String, Sort, List, SymbolicExpression, Term)}).
	 */
	private static final List<String> CLASS_FUNCTIONS = List.of("isinstance");

	/**
	 * A term that is the given one when the guard holds, and an unknown value
	 * otherwise (never precise).
	 */
	private Term when(
			Term guard,
			Term term,
			String function,
			List<Term> arguments,
			SymbolicExpression origin) {
		if (guard.equals(Term.TRUE))
			return term;
		return Term.ite(guard, term, unknown(function, term.getSort(), arguments, origin, guard));
	}

	/**
	 * A chain of cases, each with a condition and the precise encoding of the
	 * expression when it holds, ending with an unknown function of the operands
	 * when no condition holds: the encoding is precise if some condition holds
	 * (the guard of the unknown function being whether the function is used,
	 * see
	 * {@link SymbolTable#apply(String, Sort, List, SymbolicExpression, Term)}).
	 * The terms of the cases must thus be precise.
	 */
	private final class Cases {

		private final List<Term> conditions = new ArrayList<>();

		private final List<Term> terms = new ArrayList<>();

		private Cases add(
				Term condition,
				Term term) {
			if (!condition.equals(Term.FALSE)) {
				conditions.add(condition);
				terms.add(term);
			}
			return this;
		}

		private Term otherwise(
				String function,
				Sort sort,
				List<Term> arguments,
				SymbolicExpression origin) {
			Term reached = conditions.isEmpty() ? Term.FALSE
					: conditions.size() == 1 ? conditions.get(0) : Term.or(conditions.toArray(Term[]::new));
			Term result = reached.equals(Term.TRUE) ? null : unknown(function, sort, arguments, origin, reached);
			for (int i = conditions.size() - 1; i >= 0; i--)
				result = result == null ? terms.get(i) : Term.ite(conditions.get(i), terms.get(i), result);
			return result;
		}
	}

	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * Bools are numbers in Python.
	 */
	@Override
	protected void requireNumeric(
			Term term) {
		if (isValue(term))
			getSymbols().require(
					Term.or(ValueSort.isNumeric(term, getSorts().encodesFloats()), ValueSort.is(term, Sort.BOOL)));
	}

	// arithmetic

	/**
	 * Encodes a binary numeric operation of Python: between ints (or bools) it
	 * is the given operation over integers, and between numbers of which one is
	 * a float, it is the given operation over 64-bits floating point numbers,
	 * the ints being converted to floats. The operands are required to be
	 * numbers. Over other values, the result is an unknown function of the
	 * operands, specific to the given operation.
	 */
	private Term numeric(
			String function,
			ValueExpression expression,
			Term left,
			Term right,
			java.util.function.BinaryOperator<Term> ints,
			java.util.function.BinaryOperator<Term> floats) {
		if (!numericOperands(left, right))
			return havoc(expression);
		boolean values = isValue(left) || isValue(right);
		Term bothInts = Term.and(isInteger(left), isInteger(right));
		if (!values && bothInts.equals(Term.TRUE))
			return ints.apply(asInteger(left), asInteger(right));
		if (!getSorts().encodesFloats()) {
			// floating point numbers are not encoded: only ints are precise
			Term result = ints.apply(asInteger(left), asInteger(right));
			return values ? when(bothInts, ValueSort.wrap(result), function, List.of(asValue(left), asValue(right)),
					expression) : result;
		}
		if (!values)
			return floats.apply(asFloat(left, expression), asFloat(right, expression));
		// the operation over floats is computed only when some operand is a
		// float (its operands are fixed otherwise, see asFloatIf)
		Term fp = floats.apply(asFloatIf(Term.not(bothInts), left, expression),
				asFloatIf(Term.not(bothInts), right, expression));
		Term numbers = Term.and(isNumber(left), isNumber(right));
		Term result = Term.ite(bothInts, ValueSort.wrap(ints.apply(asInteger(left), asInteger(right))),
				ValueSort.wrap(fp));
		return when(numbers, result, function, List.of(asValue(left), asValue(right)), expression);
	}

	// the term as a value (see ValueSort), wrapping it if it is not one
	private static Term asValue(
			Term t) {
		return isValue(t) ? t : ValueSort.wrap(t);
	}

	private static Term lt(
			Term a,
			Term b) {
		return Term.app("<", Sort.BOOL, a, b);
	}

	private static Term neg(
			Term a) {
		return Term.app("-", Sort.INT, a);
	}

	/**
	 * Python's floor division of integers, rounding toward negative infinity
	 * (SMT-LIB's {@code div} rounds toward negative infinity for positive
	 * divisors).
	 */
	private static Term floorDiv(
			Term a,
			Term b) {
		return Term.ite(lt(Term.integer(0), b), Term.app("div", Sort.INT, a, b),
				Term.app("div", Sort.INT, neg(a), neg(b)));
	}

	/**
	 * Python's modulo of integers, taking the sign of the divisor:
	 * {@code a - b * (a // b)}.
	 */
	private static Term mod(
			Term a,
			Term b) {
		return Term.app("-", Sort.INT, a, Term.app("*", Sort.INT, b, floorDiv(a, b)));
	}

	/**
	 * Python's modulo of floats (as CPython's {@code float_rem}).
	 *
	 * @param a the dividend
	 * @param b the divisor
	 *
	 * @return the modulo
	 */
	public static double mod(
			double a,
			double b) {
		if (b == 0)
			throw new ArithmeticException("ZeroDivisionError");
		// Java's remainder of doubles is C's fmod
		double mod = a % b;
		if (mod != 0) {
			if ((b < 0) != (mod < 0))
				mod += b;
		} else
			mod = Math.copySign(0.0, b);
		return mod;
	}

	/**
	 * Python's floor division of floats (as CPython's {@code float_floor_div}).
	 *
	 * @param a the dividend
	 * @param b the divisor
	 *
	 * @return the floor division
	 */
	public static double floorDiv(
			double a,
			double b) {
		if (b == 0)
			throw new ArithmeticException("ZeroDivisionError");
		double mod = a % b;
		double div = (a - mod) / b;
		if (mod != 0 && (b < 0) != (mod < 0))
			div -= 1.0;
		if (div != 0) {
			double floordiv = Math.floor(div);
			if (div - floordiv > 0.5)
				floordiv += 1.0;
			return floordiv;
		}
		return Math.copySign(0.0, a / b);
	}

	/**
	 * Python's power of floats (as CPython's {@code float_pow}), where the
	 * cases raising exceptions (or yielding complex numbers) are not computed.
	 *
	 * @param a the base
	 * @param b the exponent
	 *
	 * @return the power
	 */
	// FIXME CPython computes the power with C's pow, whose result might differ
	// in the last bit from Java's one
	public static double pow(
			double a,
			double b) {
		if (b == 0)
			return 1.0;
		if (a == 1.0)
			return 1.0;
		if (Double.isNaN(a) || Double.isNaN(b))
			return Double.NaN;
		if (a == -1.0 && Double.isInfinite(b))
			// java yields NaN
			return 1.0;
		if (a == 0 && b < 0 && !Double.isInfinite(b))
			throw new ArithmeticException("ZeroDivisionError");
		if (a < 0 && !Double.isInfinite(a) && !Double.isInfinite(b) && b != Math.floor(b))
			throw new ArithmeticException("complex result");
		double result = StrictMath.pow(a, b);
		if (Double.isInfinite(result) && !Double.isInfinite(a) && !Double.isInfinite(b))
			throw new ArithmeticException("OverflowError");
		return result;
	}

	/**
	 * Python's true division: the quotient of the numbers as a float. The
	 * quotient of two ints is the floating point number closest to the exact
	 * quotient, that is the quotient of their conversions if they are exactly
	 * represented (magnitude at most 2^53). An int divided by a float (or vice
	 * versa) is converted to the closest float.
	 */
	private Term trueDivision(
			Term left,
			Term right,
			BinaryExpression expression) {
		if (!getSorts().encodesFloats())
			return opaque(expression);
		if (!numericOperands(left, right))
			return havoc(expression);
		Term quotient = Term.app("fp.div", Sort.FLOAT64, rne(), asFloat(left, expression),
				asFloat(right, expression));
		// the quotient of two ints is the closest float to the exact one, while
		// an int divided by a float (or vice versa) is converted to a float
		Term bothInts = Term.and(isInteger(left), isInteger(right));
		Term exact = Term.or(Term.not(bothInts), Term.and(exactlyFloat(left), exactlyFloat(right)));
		Term numbers = Term.and(isNumber(left), isNumber(right));
		boolean values = isValue(left) || isValue(right);
		Term guard = Term.and(numbers, exact);
		if (!values)
			return when(guard, quotient, "truediv", List.of(left, right), expression);
		return when(guard, ValueSort.wrap(quotient), "truediv", List.of(asValue(left), asValue(right)), expression);
	}

	/**
	 * Whether the number, if an int, is exactly represented as a float.
	 */
	private static Term exactlyFloat(
			Term t) {
		Term isInt = isInteger(t);
		if (isInt.equals(Term.FALSE))
			return Term.TRUE;
		Term i = asInteger(t);
		Term small = Term.and(Term.app("<=", Sort.BOOL, Term.integer(EXACT_FLOATS.negate()), i),
				Term.app("<=", Sort.BOOL, i, Term.integer(EXACT_FLOATS)));
		return Term.or(Term.not(isInt), small);
	}

	/**
	 * Python's power between ints with a non-negative exponent: encoded only
	 * for constant exponents (as repeated multiplications), being an unknown
	 * function of the operands otherwise.
	 */
	private Term power(
			Term left,
			Term right,
			BinaryExpression expression) {
		SymbolicExpression exponent = expression.getRight();
		if (exponent instanceof Constant && ((Constant) exponent).getValue() instanceof Number
				&& !(((Constant) exponent).getValue() instanceof Float)
				&& !(((Constant) exponent).getValue() instanceof Double)) {
			long k = ((Number) ((Constant) exponent).getValue()).longValue();
			if (k >= 0 && k <= 64)
				return numeric("pow", expression, left, right, (
						l,
						r) -> {
					Term result = Term.integer(1);
					for (long i = 0; i < k; i++)
						result = i == 0 ? l : Term.app("*", Sort.INT, result, l);
					return result;
				}, (
						l,
						r) -> getSymbols().evaluable("$py.pow", Sort.FLOAT64, List.of(l, r),
								args -> pow(args.get(0).doubleValue(), args.get(1).doubleValue()), expression));
		}
		return numeric("pow", expression, left, right,
				(
						l,
						r) -> unknown("pow", Sort.INT, List.of(l, r), expression,
								Term.not(Term.and(isInteger(left), isInteger(right)))),
				(
						l,
						r) -> getSymbols().evaluable("$py.pow", Sort.FLOAT64, List.of(l, r),
								args -> pow(args.get(0).doubleValue(), args.get(1).doubleValue()), expression));
	}

	/**
	 * Python's power with a float result: the operands are converted to floats
	 * (also the ints, as a negative int exponent yields a float).
	 */
	private Term floatPower(
			Term left,
			Term right,
			BinaryExpression expression) {
		if (!getSorts().encodesFloats())
			return opaque(expression);
		if (!numericOperands(left, right))
			return havoc(expression);
		// ints are converted to floats also when both operands are ints (as
		// CPython's long_pow does for negative exponents)
		Term numbers = Term.and(isNumber(left), isNumber(right));
		Term result = getSymbols().evaluable("$py.pow", Sort.FLOAT64,
				List.of(asFloat(left, expression), asFloat(right, expression)),
				args -> pow(args.get(0).doubleValue(), args.get(1).doubleValue()), expression);
		if (isValue(left) || isValue(right))
			return when(numbers, ValueSort.wrap(result), "floatpow", List.of(asValue(left), asValue(right)), expression);
		return when(numbers, result, "floatpow", List.of(left, right), expression);
	}

	// bitwise operators

	/**
	 * The width of the bit vectors encoding the bitwise operators over
	 * integers.
	 */
	private static final int BITS = 128;

	/**
	 * Python's bitwise operators over integers, that are unbounded:
	 * {@code &}, {@code |} and {@code ^} are computed on bit vectors of
	 * {@link #BITS} bits, exactly when both operands fit them (and then so does
	 * the result); {@code <<} and {@code >>} are a multiplication and a floor
	 * division by a power of two, exactly when the amount is a non-negative
	 * constant. Over two booleans, {@code &}, {@code |} and {@code ^} yield a
	 * boolean. Other operands (or amounts) yield an unknown function.
	 */
	private Term bitwise(
			Term left,
			Term right,
			BinaryExpression expression) {
		BinaryOperator op = expression.getOperator();
		boolean shift = op instanceof BitwiseShiftLeft || op instanceof BitwiseShiftRight;
		String name = op instanceof BitwiseAnd ? "and"
				: op instanceof BitwiseOr ? "or"
						: op instanceof BitwiseXor ? "xor" : op instanceof BitwiseShiftLeft ? "lshift" : "rshift";
		if (!shift && left.getSort().equals(Sort.BOOL) && right.getSort().equals(Sort.BOOL))
			return booleanBitwise(op, left, right);

		Term l = asValue(left), r = asValue(right);
		Term ints = Term.and(isInteger(l), isInteger(r));
		Term a = asInteger(l), b = asInteger(r);
		Term guard;
		Term value;
		if (shift) {
			Long k = amount(expression.getRight());
			if (k == null)
				return unknownBitwise(name, left, right, expression);
			Term power = Term.integer(BigInteger.ONE.shiftLeft(k.intValue()));
			guard = ints;
			value = op instanceof BitwiseShiftLeft ? Term.app("*", Sort.INT, a, power)
					: Term.app("div", Sort.INT, a, power);
		} else {
			guard = Term.and(ints, fits(a), fits(b));
			Sort bv = Sort.bitVector(BITS);
			String toBits = "(_ int2bv " + BITS + ")";
			String function = op instanceof BitwiseAnd ? "bvand" : op instanceof BitwiseOr ? "bvor" : "bvxor";
			Term unsigned = Term.app("bv2int", Sort.INT,
					Term.app(function, bv, Term.app(toBits, bv, a), Term.app(toBits, bv, b)));
			// the bits are read in two's complement
			value = Term.ite(Term.app(">=", Sort.BOOL, unsigned, Term.integer(BigInteger.ONE.shiftLeft(BITS - 1))),
					Term.app("-", Sort.INT, unsigned, Term.integer(BigInteger.ONE.shiftLeft(BITS))), unsigned);
		}
		if (!isValue(left) && !isValue(right))
			// ints and bools: an int
			return when(guard, value, name, List.of(left, right), expression);
		Cases cases = new Cases();
		if (!shift)
			cases.add(Term.and(ValueSort.is(l, Sort.BOOL), ValueSort.is(r, Sort.BOOL)),
					ValueSort.wrap(booleanBitwise(op, as(l, Sort.BOOL), as(r, Sort.BOOL))));
		return cases.add(guard, ValueSort.wrap(value)).otherwise(name, ValueSort.SORT, List.of(l, r), expression);
	}

	// the bitwise operator over two booleans
	private static Term booleanBitwise(
			BinaryOperator op,
			Term left,
			Term right) {
		if (op instanceof BitwiseAnd)
			return Term.and(left, right);
		if (op instanceof BitwiseOr)
			return Term.or(left, right);
		return Term.not(Term.eq(left, right));
	}

	// an unknown function of the operands of a bitwise operator, of the sort
	// of integers if they are ints or bools
	private Term unknownBitwise(
			String name,
			Term left,
			Term right,
			BinaryExpression expression) {
		boolean known = !isValue(left) && !isValue(right);
		return unknown(name, known ? Sort.INT : ValueSort.SORT, List.of(left, right), expression, Term.FALSE);
	}

	// whether the integer fits the bit vectors of the bitwise operators
	private static Term fits(
			Term t) {
		BigInteger half = BigInteger.ONE.shiftLeft(BITS - 1);
		return Term.and(Term.app(">=", Sort.BOOL, t, Term.integer(half.negate())),
				Term.app("<", Sort.BOOL, t, Term.integer(half)));
	}

	// the amount of a shift, if it is a non-negative constant (and not too
	// large to be written as a power of two)
	private static Long amount(
			SymbolicExpression e) {
		if (!(e instanceof Constant))
			return null;
		Object v = ((Constant) e).getValue();
		if (v instanceof Boolean)
			return (Boolean) v ? 1L : 0L;
		if (!(v instanceof Integer) && !(v instanceof Long) && !(v instanceof Short) && !(v instanceof Byte))
			return null;
		long k = ((Number) v).longValue();
		return k >= 0 && k <= 4096 ? k : null;
	}

	// comparisons

	/**
	 * Python's equality between values of possibly different kinds: numbers
	 * (ints, bools and floats) are equal if they have the same value, strings
	 * are equal to the same strings, {@code None} is only equal to itself, and
	 * values of different kinds are not equal. Equality of objects depends on
	 * their classes: it is known only for the same object (equal to itself).
	 */
	private Term equality(
			Term left,
			Term right,
			BinaryExpression expression) {
		Sort ls = left.getSort(), rs = right.getSort();
		if (!isValue(left) && !isValue(right)) {
			if (ls.equals(rs) && !ls.isFloatingPoint())
				return Term.eq(left, right);
			if (ls.isFloatingPoint() || rs.isFloatingPoint()) {
				if ((!ls.isFloatingPoint() && !ls.equals(Sort.INT) && !ls.equals(Sort.BOOL))
						|| (!rs.isFloatingPoint() && !rs.equals(Sort.INT) && !rs.equals(Sort.BOOL)))
					// e.g. a float and a string
					return Term.FALSE;
				return when(Term.and(exactlyFloat(left), exactlyFloat(right)),
						Term.app("fp.eq", Sort.BOOL, asFloat(left, expression), asFloat(right, expression)),
						"eq", List.of(left, right), expression);
			}
			if ((ls.equals(Sort.INT) || ls.equals(Sort.BOOL)) && (rs.equals(Sort.INT) || rs.equals(Sort.BOOL)))
				return Term.eq(asInteger(left), asInteger(right));
			// e.g. an int and a string
			return Term.FALSE;
		}

		Term l = asValue(left), r = asValue(right);
		Term ints = Term.and(isInteger(l), isInteger(r));
		Term numbers = Term.and(isNumber(l), isNumber(r));
		Term floats = Term.and(numbers, Term.or(isFloat(l), isFloat(r)));
		Term strings = Term.and(ValueSort.is(l, Sort.STRING), ValueSort.is(r, Sort.STRING));
		Term nones = Term.or(isNone(l), isNone(r));
		// opaque values (e.g., floats, when they are not encoded) might be
		// numbers equal to other numbers, or a NaN not equal to itself: their
		// equality is not known
		Term objects = Term.or(ValueSort.isReference(l), ValueSort.isReference(r));
		Term opaque = Term.or(ValueSort.isOpaque(l), ValueSort.isOpaque(r));
		Cases cases = new Cases().add(ints, Term.eq(asInteger(l), asInteger(r)));
		if (getSorts().encodesFloats())
			cases.add(Term.and(floats, exactlyFloat(l), exactlyFloat(r)),
					Term.app("fp.eq", Sort.BOOL, asFloatIf(floats, l, expression), asFloatIf(floats, r, expression)));
		return cases.add(strings, Term.eq(as(l, Sort.STRING), as(r, Sort.STRING)))
				.add(nones, Term.and(isNone(l), isNone(r)))
				// the same object is equal to itself (the equality of objects
				// otherwise depends on their classes)
				.add(Term.and(objects, Term.not(opaque), Term.eq(l, r)), Term.TRUE)
				// values of different kinds that are not numbers (nor objects,
				// nor opaque values)
				.add(Term.and(Term.not(numbers), Term.not(objects), Term.not(opaque)), Term.FALSE)
				.otherwise("eq", Sort.BOOL, List.of(l, r), expression);
	}

	private static Term isNone(
			Term t) {
		return Term.eq(t, ValueSort.nullValue());
	}

	/**
	 * Python's ordering comparisons ({@code <} or {@code <=}): numbers are
	 * compared by their value, strings lexicographically (by code points).
	 * Other values (e.g., bytes) are compared by an unknown function.
	 */
	private Term compare(
			String op,
			Term left,
			Term right,
			BinaryExpression expression) {
		String strOp = op.equals("<") ? "str.<" : "str.<=";
		String fpOp = op.equals("<") ? "fp.lt" : "fp.leq";
		Sort ls = left.getSort(), rs = right.getSort();
		if (ls.equals(Sort.STRING) && rs.equals(Sort.STRING))
			return Term.app(strOp, Sort.BOOL, left, right);
		if (!isValue(left) && !isValue(right)) {
			boolean ln = ls.equals(Sort.INT) || ls.equals(Sort.BOOL), rn = rs.equals(Sort.INT) || rs.equals(Sort.BOOL);
			if (ln && rn)
				return Term.app(op, Sort.BOOL, asInteger(left), asInteger(right));
			if ((ln || ls.isFloatingPoint()) && (rn || rs.isFloatingPoint()) && getSorts().encodesFloats())
				return when(Term.and(exactlyFloat(left), exactlyFloat(right)),
						Term.app(fpOp, Sort.BOOL, asFloat(left, expression), asFloat(right, expression)), "cmp" + op,
						List.of(left, right), expression);
			return unknown("cmp" + op, Sort.BOOL, List.of(left, right), expression, Term.FALSE);
		}

		Term l = asValue(left), r = asValue(right);
		Term ints = Term.and(isInteger(l), isInteger(r));
		Term strings = Term.and(ValueSort.is(l, Sort.STRING), ValueSort.is(r, Sort.STRING));
		Cases cases = new Cases().add(ints, Term.app(op, Sort.BOOL, asInteger(l), asInteger(r)))
				.add(strings, Term.app(strOp, Sort.BOOL, as(l, Sort.STRING), as(r, Sort.STRING)));
		if (getSorts().encodesFloats()) {
			Term floats = Term.and(isNumber(l), isNumber(r), exactlyFloat(l), exactlyFloat(r));
			cases.add(floats,
					Term.app(fpOp, Sort.BOOL, asFloatIf(floats, l, expression), asFloatIf(floats, r, expression)));
		}
		return cases.otherwise("cmp" + op, Sort.BOOL, List.of(l, r), expression);
	}

	/**
	 * Python's identity ({@code is}): exact when one of the operands is
	 * {@code None} (the only {@code None}) or a bool ({@code True} and
	 * {@code False} being the only bools), or both are objects (the same
	 * reference). The identity of other values (e.g., ints or strings) depends
	 * on the implementation, and it is an unknown function of them.
	 */
	private Term identity(
			Term left,
			Term right,
			BinaryExpression expression) {
		Term l = asValue(left), r = asValue(right);
		Term nones = Term.or(isNone(l), isNone(r));
		Term refs = Term.and(ValueSort.isReference(l), ValueSort.isReference(r));
		Term differentKinds = Term.or(Term.and(ValueSort.isReference(l), Term.not(ValueSort.isReference(r))),
				Term.and(ValueSort.isReference(r), Term.not(ValueSort.isReference(l))));
		// True and False are the only bools
		Term bools = Term.or(ValueSort.is(l, Sort.BOOL), ValueSort.is(r, Sort.BOOL));
		return new Cases().add(nones, Term.and(isNone(l), isNone(r)))
				.add(bools, Term.and(Term.and(ValueSort.is(l, Sort.BOOL), ValueSort.is(r, Sort.BOOL)), Term.eq(l, r)))
				.add(refs, Term.eq(l, r))
				.add(differentKinds, Term.FALSE)
				.otherwise("is", Sort.BOOL, List.of(l, r), expression);
	}

	/**
	 * Python's {@code isinstance(x, C)} for the builtin classes whose values
	 * have a sort of their own ({@code int}, of which {@code bool} is a
	 * subclass, {@code bool}, {@code float}, {@code str} and {@code NoneType});
	 * any other class (or tuple of classes) yields an unknown function of the
	 * value and the class.
	 */
	private Term isInstance(
			Term value,
			Term cls,
			BinaryExpression expression) {
		String name = className(expression.getRight());
		if (name != null) {
			if (name.equals(LibrarySpecificationProvider.INT))
				return isInteger(value);
			if (name.equals(LibrarySpecificationProvider.BOOL))
				return is(value, Sort.BOOL);
			if (name.equals(LibrarySpecificationProvider.STR))
				return is(value, Sort.STRING);
			if (name.equals(LibrarySpecificationProvider.FLOAT))
				// when floats are not encoded, a float is an opaque value
				// (as other values might be): whether it is one is not known
				return getSorts().encodesFloats() ? isFloat(value)
						: isValue(value) ? Term.and(ValueSort.isOpaque(value),
								unknown("isinstance float", Sort.BOOL, List.of(value), expression, Term.FALSE))
								: Term.FALSE;
			if (name.equals(LibrarySpecificationProvider.NONE_TYPE))
				return isValue(value) ? isNone(value) : Term.FALSE;
			// a test against a known class
			return unknown("isinstance " + name, Sort.BOOL, List.of(asValue(value)), expression, Term.FALSE);
		}
		return unknown("isinstance", Sort.BOOL, List.of(asValue(value), asValue(cls)), expression, Term.FALSE);
	}

	/**
	 * The name of the class denoted by the given expression, if it is a
	 * constant denoting a single class.
	 */
	private static String className(
			SymbolicExpression e) {
		if (!(e instanceof Constant))
			return null;
		Object value = ((Constant) e).getValue();
		if (value instanceof Unit)
			return ((Unit) value).getName();
		if (value instanceof Type && ((Type) value).isUnitType())
			return ((Type) value).asUnitType().getUnit().getName();
		Type type = e.getStaticType();
		if (type instanceof TypeTokenType && ((TypeTokenType) type).getTypes().size() == 1) {
			Type t = ((TypeTokenType) type).getTypes().iterator().next();
			if (t.isUnitType())
				return t.asUnitType().getUnit().getName();
		}
		return null;
	}

	// truthiness and conversions

	/**
	 * Python's truthiness of a value: false for {@code False}, zero, {@code ""}
	 * and {@code None}, true for other bools, numbers and strings. The
	 * truthiness of other values is an unknown function of them, except for
	 * references (e.g., lists, whose truthiness depends on their length, that
	 * might change), whose truthiness is unknown (see
	 * {@link #unknown(String, Sort, List, SymbolicExpression, Term)}).
	 */
	private Term truthiness(
			Term t,
			UnaryExpression expression) {
		Sort s = t.getSort();
		if (s.equals(Sort.BOOL))
			return t;
		if (s.equals(Sort.INT))
			return Term.not(Term.eq(t, Term.integer(0)));
		if (s.isFloatingPoint())
			return Term.not(Term.app("fp.isZero", Sort.BOOL, t));
		if (s.equals(Sort.STRING))
			return Term.not(Term.eq(Term.app("str.len", Sort.INT, t), Term.integer(0)));
		if (!isValue(t))
			return unknown("bool", Sort.BOOL, List.of(t), expression, Term.FALSE);
		Cases cases = new Cases().add(ValueSort.is(t, Sort.BOOL), ValueSort.unwrap(t, Sort.BOOL))
				.add(ValueSort.is(t, Sort.INT), Term.not(Term.eq(ValueSort.unwrap(t, Sort.INT), Term.integer(0))))
				.add(ValueSort.is(t, Sort.STRING),
						Term.not(Term.eq(Term.app("str.len", Sort.INT, ValueSort.unwrap(t, Sort.STRING)),
								Term.integer(0))))
				.add(isNone(t), Term.FALSE);
		if (getSorts().encodesFloats())
			cases.add(ValueSort.is(t, Sort.FLOAT64),
					Term.not(Term.app("fp.isZero", Sort.BOOL, ValueSort.unwrap(t, Sort.FLOAT64))))
					.add(ValueSort.is(t, Sort.FLOAT32),
							Term.not(Term.app("fp.isZero", Sort.BOOL, ValueSort.unwrap(t, Sort.FLOAT32))));
		return cases.otherwise("bool", Sort.BOOL, List.of(t), expression);
	}

	/**
	 * The int value of a bool, leaving ints unchanged.
	 */
	private static Term boolToInt(
			Term t) {
		if (t.getSort().equals(Sort.BOOL))
			return asInteger(t);
		if (isValue(t))
			return Term.ite(ValueSort.is(t, Sort.BOOL), ValueSort.wrap(asInteger(t)), t);
		return t;
	}

	/**
	 * Python's {@code float(x)} of numbers (the conversion of strings is an
	 * unknown function of them).
	 */
	private Term toPyFloat(
			Term t,
			UnaryExpression expression) {
		if (!getSorts().encodesFloats())
			return opaque(expression);
		if (t.getSort().isFloatingPoint() || t.getSort().equals(Sort.INT) || t.getSort().equals(Sort.BOOL))
			return asFloat(t, expression);
		if (!isValue(t))
			return unknown("float", Sort.FLOAT64, List.of(t), expression, Term.FALSE);
		Term numbers = isNumber(t);
		return new Cases().add(numbers, asFloat(t, expression)).otherwise("float", Sort.FLOAT64, List.of(t),
				expression);
	}

	/**
	 * Converts a 64-bits floating point number to an integer, rounding with the
	 * given mode: precise for finite numbers whose magnitude is below 2^63 (see
	 * {@link #inLongRange(Term)}).
	 */
	private Term floatToInt(
			Term fp,
			String mode) {
		Term rounded = Term.app("fp.roundToIntegral", fp.getSort(), Term.value(mode, Sort.ROUNDING_MODE), fp);
		return toIntegral(rounded, LONG);
	}

	/**
	 * Whether the floating point number is finite, with a magnitude below 2^63.
	 */
	private static Term inLongRange(
			Term fp) {
		Term bound = FloatingPoints.literal(LONG_BOUND.doubleValue());
		return Term.and(Term.app("fp.lt", Sort.BOOL, Term.app("fp.neg", fp.getSort(), bound), fp),
				Term.app("fp.lt", Sort.BOOL, fp, bound));
	}

	/**
	 * Python's {@code math.floor(x)}: the largest int not greater than the
	 * number.
	 */
	private Term floor(
			Term t,
			UnaryExpression expression) {
		Cases cases = new Cases().add(isInteger(t), asInteger(t));
		if (getSorts().encodesFloats() && (isValue(t) || t.getSort().isFloatingPoint())) {
			Term fp = asFloatIf(isFloat(t), t, expression);
			cases.add(Term.and(isFloat(t), inLongRange(fp)), floatToInt(fp, "RTN"));
		}
		return cases.otherwise("floor", Sort.INT, List.of(asValue(t)), expression);
	}

	/**
	 * Python's {@code int(x)} (with no base): ints are unchanged, bools are 0
	 * or 1, floats are truncated toward zero, and strings of decimal digits
	 * (with an optional sign) are parsed. Other strings (e.g., with spaces or
	 * underscores) yield an unknown function of them.
	 */
	private Term toPyInt(
			Term t,
			Term base,
			BinaryExpression expression) {
		if (!(expression.getRight() instanceof PyNoneConstant))
			// a base is given
			return unknown("int.base", Sort.INT, List.of(asValue(t), asValue(base)), expression, Term.FALSE);
		Cases cases = new Cases().add(isInteger(t), asInteger(t));
		if (getSorts().encodesFloats() && (isValue(t) || t.getSort().isFloatingPoint())) {
			Term fp = asFloatIf(isFloat(t), t, expression);
			cases.add(Term.and(isFloat(t), inLongRange(fp)), floatToInt(fp, "RTZ"));
		}
		if (isValue(t) || t.getSort().equals(Sort.STRING)) {
			Term s = isValue(t) ? Term.ite(ValueSort.is(t, Sort.STRING), ValueSort.unwrap(t, Sort.STRING),
					Term.string("0")) : t;
			Term[] parsed = parseInt(s);
			cases.add(Term.and(is(t, Sort.STRING), parsed[0]), parsed[1]);
		}
		return cases.otherwise("int", Sort.INT, List.of(asValue(t)), expression);
	}

	/**
	 * Parses a string of decimal digits with an optional sign, yielding the
	 * condition for the string to be one ({@code str.to_int} yields -1 for the
	 * strings that are not made of decimal digits only) and its value.
	 */
	private static Term[] parseInt(
			Term s) {
		Term len = Term.app("str.len", Sort.INT, s);
		Term first = Term.app("str.at", Sort.STRING, s, Term.integer(0));
		Term rest = Term.app("str.substr", Sort.STRING, s, Term.integer(1), Term.app("-", Sort.INT, len,
				Term.integer(1)));
		Term signed = Term.or(Term.eq(first, Term.string("-")), Term.eq(first, Term.string("+")));
		Term digits = Term.ite(signed, rest, s);
		Term value = Term.app("str.to_int", Sort.INT, digits);
		Term valid = Term.app("<=", Sort.BOOL, Term.integer(0), value);
		Term parsed = Term.ite(Term.eq(first, Term.string("-")), neg(value), value);
		return new Term[] { valid, parsed };
	}

	/**
	 * Python's {@code str(x)} (or {@code repr(x)}): the decimal digits of ints,
	 * {@code "True"} and {@code "False"} for bools, {@code "None"} for
	 * {@code None}, and the string itself for {@code str(s)}. Other values
	 * (e.g., floats, or the {@code repr} of strings) yield an unknown function
	 * of them.
	 */
	private Term str(
			Term t,
			boolean repr,
			UnaryExpression expression) {
		Sort s = t.getSort();
		if (s.equals(Sort.INT))
			return intToString(t);
		if (s.equals(Sort.BOOL))
			return boolToString(t);
		if (s.equals(Sort.STRING) && !repr)
			return t;
		String function = repr ? "repr" : "str";
		if (!isValue(t))
			return unknown(function, Sort.STRING, List.of(t), expression, Term.FALSE);
		Cases cases = new Cases().add(ValueSort.is(t, Sort.INT), intToString(ValueSort.unwrap(t, Sort.INT)))
				.add(ValueSort.is(t, Sort.BOOL), boolToString(ValueSort.unwrap(t, Sort.BOOL)))
				.add(isNone(t), Term.string("None"));
		if (!repr)
			cases.add(ValueSort.is(t, Sort.STRING), ValueSort.unwrap(t, Sort.STRING));
		return cases.otherwise(function, Sort.STRING, List.of(t), expression);
	}

	private static Term intToString(
			Term i) {
		return Term.ite(Term.app("<=", Sort.BOOL, Term.integer(0), i), Term.app("str.from_int", Sort.STRING, i),
				Term.app("str.++", Sort.STRING, Term.string("-"), Term.app("str.from_int", Sort.STRING, neg(i))));
	}

	private static Term boolToString(
			Term b) {
		return Term.ite(b, Term.string("True"), Term.string("False"));
	}

	// strings

	/**
	 * Python's {@code s[i]}: the character at the index, where negative indexes
	 * count from the end (out-of-range indexes raise {@code IndexError}, and
	 * yield an unknown value).
	 */
	private Term strGetItem(
			Term left,
			Term right,
			BinaryExpression expression) {
		Term s = asString(left, expression);
		Term i = coerce(boolToInt(right), Sort.INT, expression);
		Term len = Term.app("str.len", Sort.INT, s);
		Term index = Term.ite(lt(i, Term.integer(0)), Term.app("+", Sort.INT, i, len), i);
		Term inRange = Term.and(Term.app("<=", Sort.BOOL, Term.integer(0), index), lt(index, len));
		return when(inRange, Term.app("str.at", Sort.STRING, s, index), "str.getitem", List.of(s, i), expression);
	}

	/**
	 * Python's repetition of a string ({@code s * n}, or {@code n * s}): the
	 * empty string for non-positive counts, and the repetition for constant
	 * counts up to 32 (an unknown function of the operands otherwise).
	 */
	private Term strMult(
			Term left,
			Term right,
			BinaryExpression expression) {
		boolean stringLeft = !right.getSort().equals(Sort.STRING)
				&& (left.getSort().equals(Sort.STRING) || !left.getSort().equals(Sort.INT));
		Term s = asString(stringLeft ? left : right, expression);
		Term n = coerce(boolToInt(stringLeft ? right : left), Sort.INT, expression);
		SymbolicExpression count = stringLeft ? expression.getRight() : expression.getLeft();
		Term repeated;
		if (count instanceof Constant && ((Constant) count).getValue() instanceof Integer
				&& (Integer) ((Constant) count).getValue() <= 32) {
			int k = (Integer) ((Constant) count).getValue();
			repeated = Term.string("");
			for (int j = 0; j < k; j++)
				repeated = j == 0 ? s : Term.app("str.++", Sort.STRING, repeated, s);
			return repeated;
		}
		return new Cases().add(Term.app("<=", Sort.BOOL, n, Term.integer(0)), Term.string(""))
				.otherwise("str.mul", Sort.STRING, List.of(s, n), expression);
	}
}
