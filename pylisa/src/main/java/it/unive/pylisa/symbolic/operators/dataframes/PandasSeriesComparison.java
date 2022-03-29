package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PandasSeriesComparison implements BinaryOperator {

	private final ComparisonOperator op;

	public PandasSeriesComparison(ComparisonOperator op) {
		this.op = op;
	}

	@Override
	public String toString() {
		return this.op.toString();
	}

	public ComparisonOperator getOp() {
		return this.op;
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> left, ExternalSet<Type> right) {
		if (left.noneMatch(t -> t.equals(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF))))
			return Caches.types().mkEmptySet();
		if (right.noneMatch(t -> t.isStringType()))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES));
	}

}
