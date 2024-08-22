package it.unive.pylisa.analysis.dataframes.symbolic;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;

public class Enumerations {

	public interface TransformKind {
		int compare(
				TransformKind o);
	};

	public enum UnaryTransformKind
			implements
			TransformKind {
		UNKNOWN,
		BOTTOM,
		TO_DATETIME,
		LAMBDA,
		DROP_NA,
		DROP_COLS;

		@Override
		public int compare(
				TransformKind o) {
			if (o instanceof UnaryTransformKind)
				return this.compareTo((UnaryTransformKind) o);
			return UnaryTransformKind.class.getName().compareTo(o.getClass().getName());
		}
	}

	public enum BinaryTransformKind
			implements
			TransformKind {
		UNKNOWN,
		BOTTOM,
		FILL_NA,
		ASSIGN;

		@Override
		public int compare(
				TransformKind o) {
			if (o instanceof BinaryTransformKind)
				return this.compareTo((BinaryTransformKind) o);
			return UnaryTransformKind.class.getName().compareTo(o.getClass().getName());
		}
	}

	public interface ReshapeKind {
		int compare(
				ReshapeKind o);
	};

	public enum UnaryReshapeKind
			implements
			ReshapeKind {
		TO_GEOCODE;

		@Override
		public int compare(
				ReshapeKind o) {
			if (o instanceof UnaryReshapeKind)
				return this.compareTo((UnaryReshapeKind) o);
			return UnaryReshapeKind.class.getName().compareTo(o.getClass().getName());
		}
	}

	public interface ReduceKind {
		int compare(
				ReduceKind o);
	};

	public enum Axis
			implements
			BaseLattice<Axis> {
		BOTTOM,
		ROWS,
		COLS,
		TOP;

		@Override
		public Axis top() {
			return TOP;
		}

		@Override
		public Axis bottom() {
			return BOTTOM;
		}

		@Override
		public Axis lubAux(
				Axis other)
				throws SemanticException {
			// they can only be rows and cols
			return TOP;
		}

		@Override
		public boolean lessOrEqualAux(
				Axis other)
				throws SemanticException {
			// they can only be rows and cols
			return false;
		}

		@Override
		public StructuredRepresentation representation() {
			return new StringRepresentation(toString());
		}
	}
}
