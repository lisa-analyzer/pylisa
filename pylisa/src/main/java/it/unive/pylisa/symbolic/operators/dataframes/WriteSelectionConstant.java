package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

public class WriteSelectionConstant implements BinaryOperator {

	public static final WriteSelectionConstant INSTANCE = new WriteSelectionConstant();

	private WriteSelectionConstant() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (left.stream().noneMatch(t -> t.equals(df) || t.equals(series)))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.equals(df) || t.equals(series) || t.isNumericType() || t.isStringType()))
			return Collections.emptySet();
		return Collections.singleton(df);
	}

	@Override
	public String toString() {
		return "write_selection->";
	}
}
