package it.unive.pylisa.analysis.dataframes.operations.selection.rows;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnListSelection;
import it.unive.pylisa.symbolic.operators.dataframes.ComparisonOperator;

public class ConditionalSelection extends BooleanSelection<ConditionalSelection> {

	public static final ConditionalSelection TOP = new ConditionalSelection(new ColumnListSelection(new Names()).top(),
			ComparisonOperator.TOP,
			new ConstantPropagation().top());
	public static final ConditionalSelection BOTTOM = new ConditionalSelection(
			new ColumnListSelection(new Names()).bottom(),
			ComparisonOperator.BOT, new ConstantPropagation().bottom());

	private final ColumnListSelection cols;
	private final ComparisonOperator op;
	private final ConstantPropagation val;

	public ConditionalSelection(String colName, ComparisonOperator op, Object val) {
		this.cols = new ColumnListSelection(new Names(colName));
		this.op = op;
		this.val = new ConstantPropagation(new Constant(Untyped.INSTANCE, val, SyntheticLocation.INSTANCE));
	}

	public ConditionalSelection(ColumnListSelection cols, ComparisonOperator op, ConstantPropagation val) {
		this.cols = cols;
		this.op = op;
		this.val = val;
	}

	public ColumnListSelection getCols() {
		return cols;
	}

	public ComparisonOperator getOp() {
		return op;
	}

	public ConstantPropagation getVal() {
		return val;
	}

	@Override
	public ConditionalSelection top() {
		return TOP;
	}

	@Override
	public ConditionalSelection bottom() {
		return BOTTOM;
	}

	@Override
	public ConditionalSelection lubSameClass(ConditionalSelection other) throws SemanticException {
		return op != other.op ? top()
				: new ConditionalSelection(cols.lub(other.cols), op, val.lub(other.val));
	}

	@Override
	public ConditionalSelection wideningSameClass(ConditionalSelection other) throws SemanticException {
		return op != other.op ? top()
				: new ConditionalSelection(cols.widening(other.cols), op, val.widening(other.val));
	}

	@Override
	public boolean lessOrEqualSameClass(ConditionalSelection other) throws SemanticException {
		return op != other.op ? false
				: cols.lessOrEqual(other.cols) && val.lessOrEqual(other.val);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((cols == null) ? 0 : cols.hashCode());
		result = prime * result + ((op == null) ? 0 : op.hashCode());
		result = prime * result + ((val == null) ? 0 : val.hashCode());
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
		ConditionalSelection other = (ConditionalSelection) obj;
		if (cols == null) {
			if (other.cols != null)
				return false;
		} else if (!cols.equals(other.cols))
			return false;
		if (op != other.op)
			return false;
		if (val == null) {
			if (other.val != null)
				return false;
		} else if (!val.equals(other.val))
			return false;
		return true;
	}

	@Override
	public String toString() {
		if (isTop())
			return Lattice.TOP_STRING;

		if (isBottom())
			return Lattice.BOTTOM_STRING;

		return "filter:" + cols + op + val;
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		ConditionalSelection other = (ConditionalSelection) o;
		int cmp = op.compareTo(other.op);
		if (cmp != 0)
			return cmp;
		cmp = val.compareTo(other.val);
		if (cmp != 0)
			return cmp;
		return cols.compareTo(other.cols);
	}

	@Override
	public Names extractColumnNames() {
		return cols.extractColumnNames();
	}
}
