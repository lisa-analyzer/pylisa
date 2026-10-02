package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.value.ValueLattice;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int16Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.program.type.Int8Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.*;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.ArithmeticOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.ModuloOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.RemainderOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericFloor;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.PySyntheticLocation;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.StringAdd;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;
import org.apache.commons.lang3.tuple.Pair;

public class ConstantPropagation
		implements
		BaseNonRelationalValueDomain<ConstantPropagation>,
		Comparable<ConstantPropagation>,
		BaseLattice<ConstantPropagation>,
		ValueLattice<ConstantPropagation> {

	private static final ConstantPropagation TOP = new ConstantPropagation(null, true);
	private static final ConstantPropagation BOTTOM = new ConstantPropagation(null, false);

	private final Constant constant;

	private final boolean isTop;

	public ConstantPropagation() {
		this(null, true);
	}

	public ConstantPropagation(
			int value) {
		this(new Constant(Int32Type.INSTANCE, value, PySyntheticLocation.INSTANCE));
	}

	public ConstantPropagation(
			Constant constant) {
		this(constant, false);
	}

	private ConstantPropagation(
			Constant constant,
			boolean isTop) {
		this.constant = constant;
		this.isTop = isTop;
	}

	public Object getConstant() {
		return constant.getValue();
	}

	public <T> boolean is(
			Class<T> type) {
		return type.isInstance(getConstant());
	}

	public <T> T as(
			Class<T> type) {
		return type.cast(getConstant());
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	public StructuredRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		return new StringRepresentation(constant);
	}

	@Override
	public ConstantPropagation top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return BaseLattice.super.isTop() || (constant == null && isTop);
	}

	@Override
	public ConstantPropagation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return BaseLattice.super.isBottom() || (constant == null && !isTop);
	}

	@Override
	public ConstantPropagation lubAux(
			ConstantPropagation other)
			throws SemanticException {
		return Objects.equals(constant, other.constant) ? this : top();
	}

	@Override
	public ConstantPropagation wideningAux(
			ConstantPropagation other)
			throws SemanticException {
		return lubAux(other);
	}

	@Override
	public boolean lessOrEqualAux(
			ConstantPropagation other)
			throws SemanticException {
		return Objects.equals(constant, other.constant);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constant == null) ? 0 : constant.hashCode());
		result = prime * result + (isTop ? 1231 : 1237);
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ConstantPropagation other = (ConstantPropagation) obj;
		if (constant == null) {
			if (other.constant != null)
				return false;
		} else if (!constant.equals(other.constant))
			return false;
		if (isTop != other.isTop)
			return false;
		return true;
	}

	private static boolean isAccepted(
			Type t) {
		return t.isNumericType()
				|| t.isBooleanType()
				|| t.isStringType()
				|| t.toString().equals(LibrarySpecificationProvider.LIST)
				|| t.toString().equals(LibrarySpecificationProvider.DICT)
				|| t.toString().equals(LibrarySpecificationProvider.SLICE)
				|| t.isNullType();
	}

	@Override
	public boolean canProcess(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (expression instanceof PushInv)
			// the type approximation of a pushinv is bottom, so the below check
			// will always fail regardless of the kind of value we are tracking
			return isAccepted(expression.getStaticType());

		Set<Type> rts = null;
		try {
			rts = oracle.getRuntimeTypesOf(expression, pp);
		} catch (SemanticException e) {
			return false;
		}

		if (rts == null || rts.isEmpty())
			// if we have no runtime types, either the type domain has no type
			// information for the given expression (thus it can be anything,
			// also something that we can track) or the computation returned
			// bottom (and the whole state is likely going to go to bottom
			// anyway).
			return true;

		return rts.stream().anyMatch(ConstantPropagation::isAccepted);
	}

	@Override
	public ConstantPropagation unknownValue(
			Identifier id) {
		return BaseNonRelationalValueDomain.super.unknownValue(id);
	}

	@Override
	public ConstantPropagation evalConstant(
			Constant constant,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (isAccepted(constant.getStaticType()))
			return new ConstantPropagation(constant);
		return TOP;
	}

	@Override
	public ConstantPropagation evalUnaryExpression(
			UnaryExpression expression,
			ConstantPropagation arg,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (arg.isBottom())
			return bottom();
		if (arg.isTop())
			return top();
		UnaryOperator operator = expression.getOperator();
		if (operator == NumericNegation.INSTANCE)
			return PythonNumbers.negate(arg.constant, expression.getCodeLocation())
					.map(ConstantPropagation::new)
					.orElse(TOP);
		if (operator == NumericFloor.INSTANCE)
			return PythonNumbers.floor(arg.constant, expression.getCodeLocation())
					.map(ConstantPropagation::new)
					.orElse(TOP);

		// String constructor
		if (operator == StringConstructor.INSTANCE)
			if (arg.is(String.class))
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, arg.as(String.class), pp.getLocation()));
			else if (arg.is(Integer.class) || arg.is(Long.class))
				// str() of an integer is its decimal digits; str() of a float
				// follows Python's repr, which is not computed here
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, String.valueOf(arg.as(Number.class)),
								expression.getCodeLocation()));
		return top();
	}

	@Override
	public ConstantPropagation evalBinaryExpression(
			BinaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle) {
		BinaryOperator operator = expression.getOperator();
		if (ConstantOperations.handles(operator))
			return evalExactly(expression, left, right);
		if (operator instanceof ArithmeticOperator || operator instanceof Power) {
			if (left.isBottom() || right.isBottom())
				return bottom();
			if (left.isTop() || right.isTop())
				return top();
			if ((operator instanceof DivisionOperator || operator instanceof RemainderOperator
					|| operator instanceof ModuloOperator)
					&& PythonNumbers.isNumber(left.constant) && PythonNumbers.isZero(right.constant))
				// ZeroDivisionError: no execution continues normally
				return bottom();
			return PythonNumbers.arithmetic(operator, left.constant, right.constant, expression.getCodeLocation())
					.map(ConstantPropagation::new)
					.orElse(TOP);
		} else if (operator instanceof StringAdd)
			return stringConcat(left, right, pp);
		else if (operator instanceof StringFormat) {
			return stringFormat(left, right, pp);
		}
		if (operator instanceof StringMult)
			return stringRepeat(left, right, pp);
		if (operator instanceof ListAppend)
			return listAppend(left, right, pp);
		return top();
	}

	@Override
	public ConstantPropagation evalTernaryExpression(
			TernaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		TernaryOperator operator = (TernaryOperator) expression.getOperator();
		if (operator instanceof DictPut)
			return dictPut(left, middle, right, pp);
		return top();
	}

	@SuppressWarnings("unchecked")
	private ConstantPropagation dictPut(
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || middle.isTop() || right.isTop()) {
			return top();
		}
		DictConstant newdict = new DictConstant(pp.getLocation(), left.as(Map.class), Pair.of(middle, right));
		return new ConstantPropagation(newdict);
	}

	@SuppressWarnings("unchecked")
	private ConstantPropagation listAppend(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop() || !left.is(List.class)) {
			return TOP;
		}

		ListConstant listconst = new ListConstant(pp.getLocation(), left.as(List.class), right);
		return new ConstantPropagation(listconst);
	}

	private ConstantPropagation stringFormat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {

		if (left.isTop() || right.isTop()) {
			return TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType()) {
			return new ConstantPropagation(
					new Constant(StringType.INSTANCE, left.as(String.class) + right.as(String.class),
							pp.getLocation()));
		}
		return TOP;
	}

	/**
	 * Evaluates an operator handled by {@link ConstantOperations}: the result is
	 * the concrete one when both operands are constants and it can be
	 * computed, and top otherwise.
	 *
	 * @param expression the expression being evaluated
	 * @param left       the abstract value of the first operand
	 * @param right      the abstract value of the second operand
	 *
	 * @return the abstract value of the expression
	 */
	private ConstantPropagation evalExactly(
			BinaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation right) {
		if (left.isBottom() || right.isBottom())
			return BOTTOM;
		if (left.isTop() || right.isTop())
			return TOP;
		BinaryOperator operator = expression.getOperator();
		CodeLocation location = expression.getCodeLocation();
		if (ConstantOperations.isPredicate(operator))
			return ConstantOperations.test(operator, left.constant, right.constant)
					.map(value -> new ConstantPropagation(new Constant(BoolType.INSTANCE, value, location)))
					.orElse(TOP);
		return ConstantOperations.compute(operator, left.constant, right.constant)
				.map(value -> new ConstantPropagation(new Constant(StringType.INSTANCE, value, location)))
				.orElse(TOP);
	}

	private ConstantPropagation stringConcat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {

		if (left.isTop() || right.isTop()) {
			return TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType()) {
			return new ConstantPropagation(
					new Constant(StringType.INSTANCE, left.as(String.class) + right.as(String.class),
							pp.getLocation()));
		}
		return TOP;
	}

	private ConstantPropagation stringRepeat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop()) {
			return TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isNumericType()) {
			if (right.constant.getStaticType().asNumericType().isIntegral()) {
				// use long
				Long longRight = right.as(Number.class).longValue();
				String stringLeft = left.as(String.class);

				String repeated = stringRepeatAux(stringLeft, longRight);
				return repeated == null ? TOP
						: new ConstantPropagation(new Constant(StringType.INSTANCE, repeated, pp.getLocation()));
			}
		}
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isStringType()) {
			if (left.constant.getStaticType().asNumericType().isIntegral()) {
				// use long
				Long longLeft = left.as(Number.class).longValue();
				String stringRight = right.as(String.class);

				String repeated = stringRepeatAux(stringRight, longLeft);
				return repeated == null ? TOP
						: new ConstantPropagation(new Constant(StringType.INSTANCE, repeated, pp.getLocation()));
			}
		}
		return TOP;
	}

	/**
	 * The longest string a repetition may produce as a constant: longer
	 * results are unknown, so that the analysis does not build huge strings.
	 */
	private static final long MAX_REPEATED_LENGTH = 1 << 16;

	private String stringRepeatAux(
			String s,
			Long times) {
		if (s.isEmpty() || times <= 0)
			return "";
		if (s.length() > MAX_REPEATED_LENGTH / times)
			return null;
		StringBuilder sb = new StringBuilder();
		for (long i = 0; i < times; i++) {
			sb.append(s);
		}
		return sb.toString();
	}

	@Override
	public int compareTo(
			ConstantPropagation other) {
		if (isBottom() && !other.isBottom())
			return -1;
		else if (!isBottom() && other.isBottom())
			return 1;
		else if (isBottom())
			return 0;

		if (isTop() && !other.isTop())
			return 1;
		else if (!isTop() && other.isTop())
			return -1;
		else if (isTop())
			return 0;

		// not much we can do here..
		return Integer.compare(constant.hashCode(), other.constant.hashCode());
	}

	@Override
	public boolean knowsIdentifier(
			Identifier id) {
		return false;
	}

	@Override
	public ConstantPropagation forgetIdentifier(
			Identifier id,
			ProgramPoint pp)
			throws SemanticException {
		return null;
	}

	@Override
	public ConstantPropagation forgetIdentifiersIf(
			Predicate<Identifier> test,
			ProgramPoint pp)
			throws SemanticException {
		return null;
	}

	@Override
	public ConstantPropagation forgetIdentifiers(
			Iterable<Identifier> ids,
			ProgramPoint pp)
			throws SemanticException {
		return null;
	}

	@Override
	public ConstantPropagation pushScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		return null;
	}

	@Override
	public ConstantPropagation popScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		return null;
	}

	@Override
	public ConstantPropagation store(
			Identifier target,
			Identifier source)
			throws SemanticException {
		return null;
	}

	@Override
	public Satisfiability satisfiesAbstractValue(
			ConstantPropagation value,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (value.isTop() || value.isBottom())
			return Satisfiability.UNKNOWN;
		return ConstantOperations.truthiness(value.constant)
				.map(ConstantPropagation::satisfiability)
				.orElse(Satisfiability.UNKNOWN);
	}

	@Override
	public Satisfiability satisfiesConstant(
			Constant constant,
			ProgramPoint pp,
			SemanticOracle oracle) {
		return ConstantOperations.truthiness(constant)
				.map(ConstantPropagation::satisfiability)
				.orElse(Satisfiability.UNKNOWN);
	}

	@Override
	public Satisfiability satisfiesBinaryExpression(
			BinaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle) {
		BinaryOperator operator = expression.getOperator();
		if (!ConstantOperations.isPredicate(operator) || left.isTop() || right.isTop())
			return Satisfiability.UNKNOWN;
		return ConstantOperations.test(operator, left.constant, right.constant)
				.map(ConstantPropagation::satisfiability)
				.orElse(Satisfiability.UNKNOWN);
	}

	@Override
	public ValueEnvironment<ConstantPropagation> assumeBinaryExpression(
			ValueEnvironment<ConstantPropagation> environment,
			BinaryExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		return discardIfFalse(environment, expression, src, oracle);
	}

	@Override
	public ValueEnvironment<ConstantPropagation> assumeUnaryExpression(
			ValueEnvironment<ConstantPropagation> environment,
			UnaryExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		return discardIfFalse(environment, expression, src, oracle);
	}

	@Override
	public ValueEnvironment<ConstantPropagation> assumeConstant(
			ValueEnvironment<ConstantPropagation> environment,
			Constant expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		return discardIfFalse(environment, expression, src, oracle);
	}

	@Override
	public ValueEnvironment<ConstantPropagation> assumeIdentifier(
			ValueEnvironment<ConstantPropagation> environment,
			Identifier expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		return discardIfFalse(environment, expression, src, oracle);
	}

	/**
	 * Refines an environment with a condition that is assumed to hold: if the
	 * condition is certainly false, no execution can take the guarded path and
	 * the environment becomes bottom; otherwise it is left unchanged, which is
	 * always a sound over-approximation.
	 *
	 * @param environment the environment before the condition
	 * @param condition   the condition assumed to hold
	 * @param pp          the program point where the condition is evaluated
	 * @param oracle      the oracle for inter-domain queries
	 *
	 * @return the refined environment
	 *
	 * @throws SemanticException if the condition cannot be evaluated
	 */
	private ValueEnvironment<ConstantPropagation> discardIfFalse(
			ValueEnvironment<ConstantPropagation> environment,
			ValueExpression condition,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (satisfies(environment, condition, pp, oracle) == Satisfiability.NOT_SATISFIED)
			return environment.bottom();
		return environment;
	}

	private static Satisfiability satisfiability(
			boolean value) {
		return value ? Satisfiability.SATISFIED : Satisfiability.NOT_SATISFIED;
	}
}
