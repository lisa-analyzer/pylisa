package it.unive.pylisa.analysis.dataframes.constants;

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
import it.unive.lisa.symbolic.value.operator.Multiplication;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.common.Int32;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ConstantPropagation extends BaseNonRelationalValueDomain<ConstantPropagation> {

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
	protected ConstantPropagation lubAux(ConstantPropagation other) throws SemanticException {
		return Objects.equals(constant, other.constant) ? this : top();
	}

	@Override
	protected ConstantPropagation wideningAux(ConstantPropagation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(ConstantPropagation other) throws SemanticException {
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
				? expression.getRuntimeTypes().anyMatch(ConstantPropagation::isAccepted)
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
	protected ConstantPropagation evalUnaryExpression(UnaryOperator operator, ConstantPropagation arg,
			ProgramPoint pp) {
		if (arg.isTop())
			return top();
		if (operator == NumericNegation.INSTANCE)
			if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(Int32.INSTANCE, -1 * arg.as(Integer.class), pp.getLocation()));
		// TODO more types

		return top();
	}

	@Override
	protected ConstantPropagation evalBinaryExpression(BinaryOperator operator, ConstantPropagation left,
			ConstantPropagation right, ProgramPoint pp) {

		// TODO
		
		/*if (operator instanceof AdditionOperator)
			return left.isTop() || right.isTop() ? top() : new ConstantPropagation(left.value + right.value);
		else if (operator instanceof DivisionOperator)
			if (!left.isTop() && left.value == 0)
				return new ConstantPropagation(0);
			else if (!right.isTop() && right.value == 0)
				return bottom();
			else if (left.isTop() || right.isTop() || left.value % right.value != 0)
				return top();
			else
				return new ConstantPropagation(left.value / right.value);
		else if (operator instanceof Module)
			return left.isTop() || right.isTop() ? top() : new ConstantPropagation(left.value % right.value);
		else */if (operator instanceof Multiplication)
			return left.isTop() || right.isTop() ? top()
					: new ConstantPropagation(new Constant(Int32.INSTANCE,
							left.as(Integer.class) * right.as(Integer.class), pp.getLocation()));
		/*else if (operator instanceof SubtractionOperator)
			return left.isTop() || right.isTop() ? top() : new ConstantPropagation(left.value - right.value);*/
		else
			return top();
	}
}
