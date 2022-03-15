package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;

public class AtomicBooleanSelection extends BooleanSelection<AtomicBooleanSelection> {

	private static final AtomicBooleanSelection TOP = new AtomicBooleanSelection(new ConstantPropagation().top(),
			ComparisonOperator.TOP,
			new ConstantPropagation().top());
	private static final AtomicBooleanSelection BOTTOM = new AtomicBooleanSelection(new ConstantPropagation().bottom(),
			ComparisonOperator.BOT, new ConstantPropagation().bottom());

	private final ConstantPropagation colName;
	private final ComparisonOperator op;
	private final ConstantPropagation val;

	public AtomicBooleanSelection(String colName, ComparisonOperator op, Object val) {
		this.colName = new ConstantPropagation(new Constant(StringType.INSTANCE, colName, SyntheticLocation.INSTANCE));
		this.op = op;
		this.val = new ConstantPropagation(new Constant(Untyped.INSTANCE, val, SyntheticLocation.INSTANCE));
	}

	public AtomicBooleanSelection(ConstantPropagation colName, ComparisonOperator op, ConstantPropagation val) {
		this.colName = colName;
		this.op = op;
		this.val = val;
	}

	public ConstantPropagation getColName() {
		return colName;
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
				: new AtomicBooleanSelection(colName.lub(other.colName), op, val.lub(other.val));
	}

	@Override
	protected AtomicBooleanSelection wideningAux(AtomicBooleanSelection other) throws SemanticException {
		return op != other.op ? top()
				: new AtomicBooleanSelection(colName.widening(other.colName), op, val.widening(other.val));
	}

	@Override
	protected boolean lessOrEqualAux(AtomicBooleanSelection other) throws SemanticException {
		return op != other.op ? false
				: colName.lessOrEqual(other.colName) && val.lessOrEqual(other.val);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((colName == null) ? 0 : colName.hashCode());
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
		if (colName == null) {
			if (other.colName != null)
				return false;
		} else if (!colName.equals(other.colName))
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

		return "'" + colName + "' " + op + " " + val;
	}
}
