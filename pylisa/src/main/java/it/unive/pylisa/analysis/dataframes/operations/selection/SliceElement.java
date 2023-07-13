package it.unive.pylisa.analysis.dataframes.operations.selection;

import it.unive.pylisa.analysis.constants.ConstantPropagation;

public interface SliceElement {

	ConstantPropagation getEndIndex();

	ConstantPropagation getBeginIndex();

}
