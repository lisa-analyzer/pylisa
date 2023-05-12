package it.unive.pylisa.analysis.dataframes.operations;

import java.util.Optional;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation.Kind;

public class Transform<S extends Selection<S>> extends DataframeOperation {

	private final Kind type;
	private final S selection;
	private final boolean changeShape;
	private final Optional<Object> arg;

	public Transform(CodeLocation where, Kind type, boolean changeShape, S selection) {
		this(where, type, changeShape, selection, null);
	}

	public Transform(CodeLocation where, Kind type, boolean changeShape, S selection, Object arg) {
		super(where);
		this.type = type;
		this.selection = selection;
		this.changeShape = changeShape;
		this.arg = Optional.ofNullable(arg);
	}

	public Kind getType() {
		return type;
	}

	public S getSelection() {
		return selection;
	}

	public Optional<Object> getArg() {
		return arg;
	}

	public boolean isChangeShape() {
		return changeShape;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + (changeShape ? 1231 : 1237);
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		Transform<?> other = (Transform<?>) obj;
		if (arg == null) {
			if (other.arg != null)
				return false;
		} else if (!arg.equals(other.arg))
			return false;
		if (changeShape != other.changeShape)
			return false;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		if (type != other.type)
			return false;
		return true;
	}

	@Override
	public String toString() {
		if (arg.isEmpty())
			return type + "(" + selection + ")";
		else
			return type + "(" + selection + ", " + arg.get() + ")";
	}

	@Override
	@SuppressWarnings("unchecked")
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?> o = (Transform<?>) other;
		if (type != o.type || selection.getClass() != o.selection.getClass() || !arg.equals(o.arg)
				|| changeShape != o.changeShape)
			return false;
		return selection.lessOrEqual((S) o.selection);
	}

	@Override
	@SuppressWarnings("unchecked")
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?> o = (Transform<?>) other;
		if (type != o.type || selection.getClass() != o.selection.getClass() || changeShape != o.changeShape)
			return top();
		return new Transform<>(loc(other), type, changeShape, selection.lub((S) o.selection),
				arg.equals(o.arg) ? arg : null);
	}

	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		Transform<?> other = (Transform<?>) o;
		int cmp = type.compareTo(other.type);
		if (cmp != 0)
			return cmp;
		cmp = Boolean.compare(changeShape, other.changeShape);
		if (cmp != 0)
			return cmp;
		cmp = selection.compareTo(other.selection);
		if (cmp != 0)
			return cmp;
		// not much we can do here..
		return Integer.compare(arg.hashCode(), other.arg.hashCode());
	}
}
