package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.TransformKind;
import java.util.Optional;

public class Transform<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final TransformKind type;
	private final Axis axis;
	private final DataframeSelection<R, C> selection;
	private final boolean yieldsNewStructure;
	private final Optional<Object> arg;

	public Transform(CodeLocation where,
			int index,
			TransformKind type,
			Axis axis,
			boolean yieldsNewStructure,
			DataframeSelection<R, C> selection) {
		this(where, index, type, axis, yieldsNewStructure, selection, null);
	}

	public Transform(CodeLocation where,
			int index,
			TransformKind type,
			Axis axis,
			boolean yieldsNewStructure,
			DataframeSelection<R, C> selection,
			Object arg) {
		super(where, index);
		this.type = type;
		this.axis = axis;
		this.selection = selection;
		this.yieldsNewStructure = yieldsNewStructure;
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

	public boolean yieldsNewStructure() {
		return yieldsNewStructure;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + (yieldsNewStructure ? 1231 : 1237);
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
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
		if (yieldsNewStructure != other.yieldsNewStructure)
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
		if (yieldsNewStructure)
			ret += " *NEW_STRUCTURE*";
		return ret;
	}

	@Override
	@SuppressWarnings("unchecked")
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?, ?> o = (Transform<?, ?>) other;
		if (type != o.type || axis != o.axis || selection.getClass() != o.selection.getClass() || !arg.equals(o.arg)
				|| yieldsNewStructure != o.yieldsNewStructure)
			return false;
		return selection.lessOrEqual((DataframeSelection<R, C>) o.selection);
	}

	@Override
	@SuppressWarnings("unchecked")
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?, ?> o = (Transform<?, ?>) other;
		if (type != o.type || axis != o.axis || selection.getClass() != o.selection.getClass()
				|| yieldsNewStructure != o.yieldsNewStructure)
			return top();
		return new Transform<>(where, index, type, axis, yieldsNewStructure,
				selection.lub((DataframeSelection<R, C>) o.selection),
				arg.equals(o.arg) ? arg : null);
	}

	@Override
	protected int compareToSameOperation(DataframeOperation o) {
		Transform<?, ?> other = (Transform<?, ?>) o;
		int cmp = type.compare(other.type);
		if (cmp != 0)
			return cmp;
		cmp = axis.compareTo(other.axis);
		if (cmp != 0)
			return cmp;
		cmp = Boolean.compare(yieldsNewStructure, other.yieldsNewStructure);
		if (cmp != 0)
			return cmp;
		cmp = selection.compareTo(other.selection);
		if (cmp != 0)
			return cmp;
		// not much we can do here..
		return Integer.compare(arg.hashCode(), other.arg.hashCode());
	}

	@Override
	@SuppressWarnings("unchecked")
	protected DataframeOperation wideningSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?, ?> o = (Transform<?, ?>) other;
		if (type != o.type || axis != o.axis || selection.getClass() != o.selection.getClass()
				|| yieldsNewStructure != o.yieldsNewStructure)
			return top();
		return new Transform<>(where, index, type, axis, yieldsNewStructure,
				selection.widening((DataframeSelection<R, C>) o.selection),
				arg.equals(o.arg) ? arg : null);
	}
}
