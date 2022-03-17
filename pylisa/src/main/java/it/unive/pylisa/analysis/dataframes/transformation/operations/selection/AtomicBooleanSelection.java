package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.Names;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;

public class AtomicBooleanSelection extends BooleanSelection<AtomicBooleanSelection> {

	private static final AtomicBooleanSelection TOP = new AtomicBooleanSelection(new ColumnListSelection(new Names()).top(),
			ComparisonOperator.TOP,
			new ConstantPropagation().top());
	private static final AtomicBooleanSelection BOTTOM = new AtomicBooleanSelection(new ColumnListSelection(new Names()).bottom(),
			ComparisonOperator.BOT, new ConstantPropagation().bottom());

	private final ColumnListSelection cols;
	private final ComparisonOperator op;
	private final ConstantPropagation val;

	public AtomicBooleanSelection(String colName, ComparisonOperator op, Object val) {
		this.cols = new ColumnListSelection(new Names(colName));
		this.op = op;
		this.val = new ConstantPropagation(new Constant(Untyped.INSTANCE, val, SyntheticLocation.INSTANCE));
	}

	public AtomicBooleanSelection(ColumnListSelection cols, ComparisonOperator op, ConstantPropagation val) {
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
	public AtomicBooleanSelection top() {
		return TOP;
	}

	@Override
	public AtomicBooleanSelection bottom() {
		return BOTTOM;
	}

	@Override
	protected AtomicBooleanSelection lubAux(AtomicBooleanSelection other) throws SemanticException {
		return op != other.op ? top()
				: new AtomicBooleanSelection(cols.lub(other.cols), op, val.lub(other.val));
	}

	@Override
	protected AtomicBooleanSelection wideningAux(AtomicBooleanSelection other) throws SemanticException {
		return op != other.op ? top()
				: new AtomicBooleanSelection(cols.widening(other.cols), op, val.widening(other.val));
	}

	@Override
	protected boolean lessOrEqualAux(AtomicBooleanSelection other) throws SemanticException {
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
		AtomicBooleanSelection other = (AtomicBooleanSelection) obj;
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

		return "'" + cols + "' " + op + " " + val;
	}
}
