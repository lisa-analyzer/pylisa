package it.unive.ros.lisa.analysis.constants;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int16Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.program.type.Int8Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushInv;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.ArithmeticOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.ModuloOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.RemainderOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.StringAdd;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import it.unive.ros.lisa.symbolic.operators.ros.ROSTopicNameExpansion;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

public class ConstantPropagation
		implements
		BaseNonRelationalValueDomain<ConstantPropagation>,
		Comparable<ConstantPropagation> {

	private static final ConstantPropagation TOP = new ConstantPropagation(null, true);
	private static final ConstantPropagation BOTTOM = new ConstantPropagation(null, false);

	private final Constant constant;

	private final boolean isTop;

	public ConstantPropagation() {
		this(null, true);
	}

	public ConstantPropagation(
			int value) {
		this(new Constant(Int32Type.INSTANCE, value, SyntheticLocation.INSTANCE));
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
		return BaseNonRelationalValueDomain.super.isTop() || (constant == null && isTop);
	}

	@Override
	public ConstantPropagation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return BaseNonRelationalValueDomain.super.isBottom() || (constant == null && !isTop);
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
				|| t.isStringType()
				|| t.toString().equals(LibrarySpecificationProvider.LIST)
				|| t.toString().equals(LibrarySpecificationProvider.DICT)
				|| t.toString().equals(LibrarySpecificationProvider.SLICE)
				|| t.isNullType();
	}

	@Override
	public boolean canProcess(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (expression instanceof PushInv)
			// the type approximation of a pushinv is bottom, so the below check
			// will always fail regardless of the kind of value we are tracking
			return isAccepted(expression.getStaticType());

		Set<Type> rts = null;
		try {
			rts = oracle.getRuntimeTypesOf(expression, pp, oracle);
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
	public ConstantPropagation evalNullConstant(
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return new ConstantPropagation(new PyNoneConstant(pp.getLocation()));
	}

	@Override
	public ConstantPropagation evalNonNullConstant(
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
			UnaryOperator operator,
			ConstantPropagation arg,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (arg.isTop())
			return top();
		if (operator == NumericNegation.INSTANCE)
			if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, -1 * arg.as(Integer.class), pp.getLocation()));
			else if (arg.is(Float.class))
				return new ConstantPropagation(
						new Constant(Float32Type.INSTANCE, -1 * arg.as(Float.class), pp.getLocation()));

		// String constructor
		if (operator == StringConstructor.INSTANCE)
			if (arg.is(String.class))
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, arg.as(String.class), pp.getLocation()));
			else if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, arg.as(Integer.class), pp.getLocation()));
		return top();
	}

	@Override
	public ConstantPropagation evalBinaryExpression(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (operator instanceof ArithmeticOperator) {
			if (left.isTop() || right.isTop() || !left.constant.getStaticType().isNumericType()
					|| !right.constant.getStaticType().isNumericType())
				return top();

			Constant c;
			if (operator instanceof AdditionOperator)
				c = sum(left, right, pp);
			else if (operator instanceof DivisionOperator)
				if ((right.is(Integer.class) && right.as(Integer.class) == 0)
						|| (right.is(Float.class) && right.as(Float.class) == 0f))
					return bottom();
				else
					c = div(left, right, pp);
			else if (operator instanceof RemainderOperator || operator instanceof ModuloOperator)
				if ((right.is(Integer.class) && right.as(Integer.class) == 0)
						|| (right.is(Float.class) && right.as(Float.class) == 0f))
					return bottom();
				else
					c = rem(left, right, pp);
			else if (operator instanceof MultiplicationOperator)
				c = mul(left, right, pp);
			else if (operator instanceof SubtractionOperator)
				c = sub(left, right, pp);
			else if (operator instanceof Power)
				return power(left, right, pp);
			else
				return top();
			return new ConstantPropagation(c);
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
			TernaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (operator instanceof DictPut)
			return dictPut(left, middle, right, pp);
		if (operator instanceof ROSTopicNameExpansion)
			return ROSTopicNameExpansion(left, middle, right, pp);
		return top();
	}

	private ConstantPropagation ROSTopicNameExpansion(
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || middle.isTop() || right.isTop())
			return top();
		String topicName = left.as(String.class);
		String namespace = middle.as(String.class);
		String nodeName = right.as(String.class);
		// 1. remove prefix
		if (namespace.startsWith("rostopic://")) {
			namespace.replace("rostopic://", "");
		}
		// 2. fix namespace
		if (!namespace.startsWith("/")) {
			namespace = "/" + namespace;
		}
		// 3. substitute ~
		topicName = topicName.replace("~", !namespace.equals("/") ? (namespace + "/" + nodeName) : "/" + nodeName);

		// 4. check if we can prepend the namespace on topicName
		if (!topicName.startsWith("/") && !topicName.startsWith("{namespace}") && !topicName.startsWith("{ns}")) {
			topicName = !namespace.equals("/") ? (namespace + "/" + topicName) : ("/" + topicName);
		}
		// 4. apply substitution
		topicName = topicName.replace("{namespace}", namespace).replace("{ns}", namespace).replace("{node}", nodeName);
		return new ConstantPropagation(new Constant(StringType.INSTANCE, topicName, pp.getLocation()));
	}

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

	private ConstantPropagation listAppend(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop() || !left.is(List.class)) {
			return TOP;
		}
		Lattice<?> tail;

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

	private Constant div(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			Integer l = left.as(Integer.class);
			Integer r = right.as(Integer.class);
			c = l % r == 0
					? new Constant(Int32Type.INSTANCE, l / r, pp.getLocation())
					: new Constant(Float32Type.INSTANCE, l / (float) r, pp.getLocation());
		} else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) / right.as(Integer.class), pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) / right.as(Float.class), pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) / right.as(Float.class), pp.getLocation());
		return c;
	}

	private Constant rem(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			Integer l = left.as(Integer.class);
			Integer r = right.as(Integer.class);
			c = l % r == 0
					? new Constant(Int32Type.INSTANCE, l % r, pp.getLocation())
					: new Constant(Float32Type.INSTANCE, l % (float) r, pp.getLocation());
		} else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) % right.as(Integer.class), pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) % right.as(Float.class), pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) % right.as(Float.class), pp.getLocation());
		return c;
	}

	private Constant sum(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) + right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) + right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) + right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) + right.as(Float.class),
					pp.getLocation());
		return c;
	}

	private Constant sub(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) - right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) - right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) - right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) - right.as(Float.class),
					pp.getLocation());
		return c;
	}

	private Constant mul(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) * right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) * right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) * right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) * right.as(Float.class),
					pp.getLocation());
		return c;
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
				Long longRight = right.as(Integer.class).longValue();
				String stringLeft = left.as(String.class);

				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, stringRepeatAux(stringLeft, longRight), pp.getLocation()));
			}
		}
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isStringType()) {
			if (left.constant.getStaticType().asNumericType().isIntegral()) {
				// use long
				Long longLeft = left.as(Integer.class).longValue();
				String stringRight = right.as(String.class);

				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, stringRepeatAux(stringRight, longLeft), pp.getLocation()));
			}
		}
		return TOP;
	}

	private ConstantPropagation power(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop()) {
			return TOP;
		}
		// TODO: handle overflow (?)
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isNumericType()) {
			NumericType superType = left.constant.getStaticType().asNumericType()
					.supertype(right.constant.getStaticType().asNumericType());
			// Class<? extends Number> type = getJavaClassFor(superType);
			if (superType.is8Bits()) {
				return new ConstantPropagation(
						new Constant(Int8Type.INSTANCE,
								(byte) (Math.pow((double) left.as(Byte.class), (double) right.as(Byte.class))),
								pp.getLocation()));
			}
			if (superType.is16Bits()) {
				return new ConstantPropagation(
						new Constant(Int16Type.INSTANCE,
								(short) (Math.pow((double) left.as(Short.class), (double) right.as(Short.class))),
								pp.getLocation()));
			}
			if (superType.is32Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float32Type.INSTANCE,
									(float) (Math.pow((double) left.as(Float.class), (double) right.as(Float.class))),
									pp.getLocation()));
				} else {
					if (right.as(Integer.class) < 0) {
						return new ConstantPropagation(
								new Constant(Float32Type.INSTANCE, (float) (Math.pow((double) left.as(Integer.class),
										(double) right.as(Integer.class))), pp.getLocation()));

					} else {
						return new ConstantPropagation(
								new Constant(Int32Type.INSTANCE, (int) (Math.pow((double) left.as(Integer.class),
										(double) right.as(Integer.class))), pp.getLocation()));

					}
				}
			}
			if (superType.is64Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float64Type.INSTANCE, Math.pow(left.as(Double.class), right.as(Double.class)),
									pp.getLocation()));
				} else {
					return new ConstantPropagation(
							new Constant(Int64Type.INSTANCE,
									(long) (Math.pow((double) left.as(Long.class), (double) right.as(Long.class))),
									pp.getLocation()));
				}
			}
		}
		return TOP;
	}

	private String stringRepeatAux(
			String s,
			Long times) {
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
}
