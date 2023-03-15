package it.unive.pylisa.analysis.constants;

import java.util.Objects;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.common.*;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ConstantPropagation extends BaseNonRelationalValueDomain<ConstantPropagation>
		implements Comparable<ConstantPropagation> {

	private static final ConstantPropagation TOP = new ConstantPropagation(null, true);
	private static final ConstantPropagation BOTTOM = new ConstantPropagation(null, false);

	private final Constant constant;

	private final boolean isTop;

	public ConstantPropagation() {
		this(null, true);
	}

	public ConstantPropagation(Constant constant) {
		this(constant, false);
	}

	private ConstantPropagation(Constant constant, boolean isTop) {
		this.constant = constant;
		this.isTop = isTop;
	}

	public Object getConstant() {
		return constant.getValue();
	}

	public <T> boolean is(Class<T> type) {
		return type.isInstance(getConstant());
	}

	public <T> T as(Class<T> type) {
		return type.cast(getConstant());
	}

	public DomainRepresentation representation() {
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
		return super.isTop() || (constant == null && isTop);
	}

	@Override
	public ConstantPropagation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return super.isBottom() || (constant == null && !isTop);
	}

	@Override
	public ConstantPropagation lubAux(ConstantPropagation other) throws SemanticException {
		return Objects.equals(constant, other.constant) ? this : top();
	}

	@Override
	public ConstantPropagation wideningAux(ConstantPropagation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	public boolean lessOrEqualAux(ConstantPropagation other) throws SemanticException {
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
	public boolean equals(Object obj) {
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

	private static boolean isAccepted(Type t) {
		return t.isNumericType()
				|| t.isStringType()
				|| t.toString().equals(LibrarySpecificationProvider.LIST)
				|| t.toString().equals(LibrarySpecificationProvider.DICT)
				|| t.toString().equals(LibrarySpecificationProvider.SLICE);
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return canProcess(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes(null).stream().anyMatch(ConstantPropagation::isAccepted)
				: isAccepted(expression.getStaticType());
	}

	@Override
	public ConstantPropagation evalNullConstant(ProgramPoint pp) throws SemanticException {
		return TOP;
	}

	@Override
	public ConstantPropagation evalNonNullConstant(Constant constant, ProgramPoint pp) throws SemanticException {
		if (isAccepted(constant.getStaticType()))
			return new ConstantPropagation(constant);
		return TOP;
	}

	@Override
	public ConstantPropagation evalUnaryExpression(UnaryOperator operator, ConstantPropagation arg,
			ProgramPoint pp) {
		if (arg.isTop())
			return top();
		if (operator == NumericNegation.INSTANCE)
			if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, -1 * arg.as(Integer.class), pp.getLocation()));
		// TODO more types

		return top();
	}

	@Override
	public ConstantPropagation evalBinaryExpression(BinaryOperator operator, ConstantPropagation left,
			ConstantPropagation right, ProgramPoint pp) {
		// test int addition propagation
		if (operator instanceof AdditionOperator) {
			return add(left, right, pp);

		}
		// TODO
		/*
		 * if (operator instanceof AdditionOperator) return left.isTop() ||
		 * right.isTop() ? top() : new ConstantPropagation(left.value +
		 * right.value); else if (operator instanceof DivisionOperator) if
		 * (!left.isTop() && left.value == 0) return new ConstantPropagation(0);
		 * else if (!right.isTop() && right.value == 0) return bottom(); else if
		 * (left.isTop() || right.isTop() || left.value % right.value != 0)
		 * return top(); else return new ConstantPropagation(left.value /
		 * right.value); else if (operator instanceof Module) return
		 * left.isTop() || right.isTop() ? top() : new
		 * ConstantPropagation(left.value % right.value); else
		 */if (operator instanceof MultiplicationOperator)
			return left.isTop() || right.isTop() ? top()
					: new ConstantPropagation(new Constant(Int32Type.INSTANCE,
							left.as(Integer.class) * right.as(Integer.class), pp.getLocation()));
		/*
		 * else if (operator instanceof SubtractionOperator) return left.isTop()
		 * || right.isTop() ? top() : new ConstantPropagation(left.value -
		 * right.value);
		 */
		else
			return top();
	}

	@Override
	public int compareTo(ConstantPropagation other) {
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


	public ConstantPropagation add(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
		if (left.isTop() || right.isTop()) {
			return TOP;
		}
		// NUMERIC
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isNumericType()) {
			NumericType superType = left.constant.getStaticType().asNumericType().supertype(right.constant.getStaticType().asNumericType());
			//Class<? extends Number> type = getJavaClassFor(superType);
			if (superType.is8Bits()) {
				return new ConstantPropagation(
						new Constant(Int8Type.INSTANCE, left.as(Byte.class) + right.as(Byte.class),pp.getLocation()));
			}
			if (superType.is16Bits()) {
				return new ConstantPropagation(
						new Constant(Int16Type.INSTANCE, left.as(Short.class) + right.as(Short.class),pp.getLocation()));
			}
			if (superType.is32Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float32Type.INSTANCE, left.as(Float.class) + right.as(Float.class), pp.getLocation()));
				} else {
					if (left.constant.getStaticType().asNumericType().isIntegral()) {

					}
					return new ConstantPropagation(
							new Constant(Int32Type.INSTANCE, left.as(Integer.class) + right.as(Integer.class), pp.getLocation()));
				}
			}
			if (superType.is64Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float64Type.INSTANCE, left.as(Double.class) + right.as(Double.class), pp.getLocation()));
				} else {
					return new ConstantPropagation(
							new Constant(Int64Type.INSTANCE, left.as(Long.class) + right.as(Long.class),pp.getLocation()));
				}
			}

			//Constant _left = new Constant(superType.)
			//NumericType _right =
			return TOP;
		}
		// STRING
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType()) {
			return new ConstantPropagation(
					new Constant(StringType.INSTANCE, left.as(String.class) + right.as(String.class),pp.getLocation()));
		}
		return TOP;
	}


	public static Class<? extends Number> getJavaClassFor(NumericType numericType) {
		if (numericType.is8Bits()) return Byte.class;
		if (numericType.is16Bits()) return Short.class;
		if (numericType.is32Bits() && numericType.isIntegral()) return Integer.class;
		if (numericType.is32Bits() && !numericType.isIntegral()) return Float.class;
		if (numericType.is64Bits() && numericType.isIntegral()) return Long.class;
		return Double.class;
	}

}
