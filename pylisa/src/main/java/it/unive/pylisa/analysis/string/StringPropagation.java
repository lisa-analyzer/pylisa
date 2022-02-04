package it.unive.pylisa.analysis.string;

import java.util.Objects;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.type.Type;

public class StringPropagation extends BaseNonRelationalValueDomain<StringPropagation> {

	private static final StringPropagation TOP = new StringPropagation(null, true);
	private static final StringPropagation BOTTOM = new StringPropagation(null, false);

	private final String constant;

	private final boolean isTop;

	public StringPropagation() {
		this(null, true);
	}

	private StringPropagation(String constant) {
		this(constant, false);
	}

	private StringPropagation(String constant, boolean isTop) {
		this.constant = constant;
		this.isTop = isTop;
	}

	public String getConstant() {
		return constant;
	}

	@Override
	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		return new StringRepresentation(constant);
	}

	@Override
	public StringPropagation top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return super.isTop() || (constant == null && isTop);
	}

	@Override
	public StringPropagation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return super.isBottom() || (constant == null && !isTop);
	}

	@Override
	protected StringPropagation lubAux(StringPropagation other) throws SemanticException {
		return Objects.equals(constant, other.constant) ? this : top();
	}

	@Override
	protected StringPropagation wideningAux(StringPropagation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(StringPropagation other) throws SemanticException {
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
		StringPropagation other = (StringPropagation) obj;
		if (constant == null) {
			if (other.constant != null)
				return false;
		} else if (!constant.equals(other.constant))
			return false;
		if (isTop != other.isTop)
			return false;
		return true;
	}

	@Override
	protected StringPropagation evalNonNullConstant(Constant constant, ProgramPoint pp) throws SemanticException {
		if (constant.getStaticType().isStringType())
			return new StringPropagation(constant.getValue().toString());
		return super.evalNonNullConstant(constant, pp);
	}

	@Override
	protected StringPropagation evalBinaryExpression(BinaryOperator operator, StringPropagation left,
			StringPropagation right, ProgramPoint pp) throws SemanticException {
		if (operator instanceof StringConcat) {
			if (left.isTop() || right.isTop())
				return top();
			return new StringPropagation(left.constant + right.constant);
		}

		return super.evalBinaryExpression(operator, left, right, pp);
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return id.hasRuntimeTypes()
				? id.getRuntimeTypes().anyMatch(Type::isStringType)
				: id.getStaticType().isStringType() || id.getStaticType().isUntyped();
	}
}
