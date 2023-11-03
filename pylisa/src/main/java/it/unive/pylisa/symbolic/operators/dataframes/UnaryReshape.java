package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.UnaryReshapeKind;
import java.util.Collections;
import java.util.Set;

public class UnaryReshape implements UnaryOperator, DataframeOperator {

	private final UnaryReshapeKind type;

	private final int index;

	@Override
	public int getIndex() {
		return index;
	}

	public UnaryReshape(
			int index,
			UnaryReshapeKind type) {
		this.index = index;
		this.type = type;
	}

	public UnaryReshapeKind getKind() {
		return type;
	}

	@Override
	public String toString() {
		return "reshape(" + type + ")";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (argument.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		return Collections.singleton(series);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		UnaryReshape other = (UnaryReshape) obj;
		if (type != other.type)
			return false;
		return true;
	}
}
