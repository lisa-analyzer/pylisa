package it.unive.pylisa.analysis.constants;

import java.util.Objects;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.ArithmeticOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.ModuloOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.RemainderOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ConstantPropagation implements
		BaseNonRelationalValueDomain<ConstantPropagation>,
		Comparable<ConstantPropagation> {

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

	@Override
	public String toString() {
		return representation().toString();
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
			else if (arg.is(Float.class))
				return new ConstantPropagation(
						new Constant(Float32Type.INSTANCE, -1 * arg.as(Float.class), pp.getLocation()));
		return top();
	}

	@Override
	public ConstantPropagation evalBinaryExpression(BinaryOperator operator, ConstantPropagation left,
			ConstantPropagation right, ProgramPoint pp) {
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
			else
				return top();
			return new ConstantPropagation(c);
		} else
			return top();
	}

	private Constant div(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
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

	private Constant rem(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
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

	private Constant sum(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
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

	private Constant sub(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
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

	private Constant mul(ConstantPropagation left, ConstantPropagation right, ProgramPoint pp) {
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
}
