package it.unive.pylisa.analysis.dataframes.symbolic;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.analysis.dataframes.symbolic.aux.ComparisonOperator;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class SeriesComparison implements BinaryOperator, DataframeOperator {

	private final ComparisonOperator op;

	private final int index;

	public SeriesComparison(
			int index,
			ComparisonOperator op) {
		this.index = index;
		this.op = op;
	}

	@Override
	public int getIndex() {
		return index;
	}

	@Override
	public String toString() {
		return this.op.toString();
	}

	public ComparisonOperator getOp() {
		return this.op;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		if (left.stream().noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Collections.emptySet();
		if (right.stream().noneMatch(t -> t.isStringType()))
			return Collections.emptySet();
		return Collections.singleton(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES));
	}

}
