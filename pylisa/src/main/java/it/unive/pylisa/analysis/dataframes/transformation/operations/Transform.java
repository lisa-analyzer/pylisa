package it.unive.pylisa.analysis.dataframes.transformation.operations;

import java.util.Optional;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.Selection;
import it.unive.pylisa.symbolic.operators.ApplyTransformation.Kind;

public class Transform<S extends Selection<S>> extends DataframeOperation {

	private final Kind type;
	private final S selection;
	private final Optional<Object> arg;

	public Transform(CodeLocation where, Kind type, S selection) {
		this(where, type, selection, null);
	}

	public Transform(CodeLocation where, Kind type, S selection, Object arg) {
		super(where);
		this.type = type;
		this.selection = selection;
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

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
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
		Transform<?> other = (Transform<?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		if (type != other.type)
			return false;
		if (arg == null) {
			if (other.arg != null)
				return false;
		} else if (!arg.equals(other.arg))
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
		if (type != o.type || selection.getClass() != o.selection.getClass() || !arg.equals(o.arg))
			return false;
		return selection.lessOrEqual((S) o.selection);
	}

	@Override
	@SuppressWarnings("unchecked")
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		Transform<?> o = (Transform<?>) other;
		if (type != o.type || selection.getClass() != o.selection.getClass())
			return top();
		return new Transform<>(loc(other), type, selection.lub((S) o.selection), arg.equals(o.arg) ? arg : null);
	}
}
