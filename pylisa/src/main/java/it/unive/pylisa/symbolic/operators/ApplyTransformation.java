package it.unive.pylisa.symbolic.operators;

import java.util.Optional;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ApplyTransformation implements UnaryOperator, DataframeOperatorWithSideEffects {

	public enum Kind {
		UNKNOWN,
		BOTTOM,
		TO_DATETIME,
		TO_GEOCODE,
		LAMBDA,
	}

	private final Kind type;

	private final Optional<Object> arg;

	public ApplyTransformation(Kind type) {
		this(type, null);
	}

	public ApplyTransformation(Kind type, Object arg) {
		this.type = type;
		this.arg = Optional.ofNullable(arg);
	}

	public Kind getKind() {
		return type;
	}

	public Optional<Object> getArg() {
		return arg;
	}

	@Override
	public String toString() {
		if (arg.isEmpty())
			return "apply(" + type + ")";
		else
			return "apply(" + type + ", " + arg.get() + ")";
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> argument) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (argument.noneMatch(t -> t.equals(series)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(series);
	}

	@Override
	public SymbolicExpression getDataFrame(SymbolicExpression container) {
		return ((UnaryExpression) container).getExpression();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		ApplyTransformation other = (ApplyTransformation) obj;
		if (arg == null) {
			if (other.arg != null)
				return false;
		} else if (!arg.equals(other.arg))
			return false;
		if (type != other.type)
			return false;
		return true;
	}
}
