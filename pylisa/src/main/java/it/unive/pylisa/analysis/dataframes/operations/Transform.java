package it.unive.pylisa.analysis.dataframes.operations;

import java.util.Optional;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.Axis;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.TransformKind;

public class Transform<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final TransformKind type;
	private final Axis axis;
	private final DataframeSelection<R, C> selection;
	private final Optional<Object> arg;

	public Transform(
			CodeLocation where,
			int index,
			TransformKind type,
			Axis axis,
			DataframeSelection<R, C> selection) {
		this(where, index, type, axis, selection, null);
	}

	public Transform(
			CodeLocation where,
			int index,
			TransformKind type,
			Axis axis,
			DataframeSelection<R, C> selection,
			Object arg) {
		super(where, index);
		this.type = type;
		this.axis = axis;
		this.selection = selection;
		this.arg = Optional.ofNullable(arg);
	}

	public TransformKind getType() {
		return type;
	}

	public Axis getAxis() {
		return axis;
	}

	public DataframeSelection<R, C> getSelection() {
		return selection;
	}

	public Optional<Object> getArg() {
		return arg;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Transform<?, ?> other = (Transform<?, ?>) obj;
		if (arg == null) {
			if (other.arg != null)
				return false;
		} else if (!arg.equals(other.arg))
			return false;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		if (type != other.type)
			return false;
		if (axis != other.axis)
			return false;
		return true;
	}

	@Override
	public String toString() {
		String ret = "transform " + axis + ": " + type + "(" + selection;
		if (arg.isPresent())
			ret += ", " + arg.get();
		ret += ")";
		return ret;
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Transform<?, ?> o = (Transform<?, ?>) other;
		if (type != o.type || !arg.equals(o.arg))
			return false;
		return axis.lessOrEqual(o.axis) && selection.lessOrEqual(o.selection);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Transform<?, ?> o = (Transform<?, ?>) other;
		if (type != o.type || !arg.equals(o.arg))
			return top();
		return new Transform<>(where, index, type, axis.lub(o.axis), selection.lub(o.selection), arg);
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		Transform<?, ?> other = (Transform<?, ?>) o;
		int cmp = type.compare(other.type);
		if (cmp != 0)
			return cmp;
		cmp = axis.compareTo(other.axis);
		if (cmp != 0)
			return cmp;
		cmp = selection.compareTo(other.selection);
		if (cmp != 0)
			return cmp;
		// not much we can do here..
		return Integer.compare(arg.hashCode(), other.arg.hashCode());
	}
}
