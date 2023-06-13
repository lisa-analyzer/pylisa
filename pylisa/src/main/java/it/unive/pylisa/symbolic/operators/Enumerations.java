package it.unive.pylisa.symbolic.operators;

public class Enumerations {

	public interface TransformKind {
		int compare(TransformKind o);
	};

	public enum UnaryKind implements TransformKind {
		UNKNOWN,
		BOTTOM,
		TO_DATETIME,
		TO_GEOCODE,
		LAMBDA,
		DROP_NA,
		DROP_COLS;

		@Override
		public int compare(TransformKind o) {
			if (o instanceof UnaryKind)
				return this.compareTo((UnaryKind) o);
			return UnaryKind.class.getName().compareTo(o.getClass().getName());
		}
	}

	public enum BinaryKind implements TransformKind {
		UNKNOWN,
		BOTTOM,
		FILL_NA,
		ASSIGN;

		@Override
		public int compare(TransformKind o) {
			if (o instanceof BinaryKind)
				return this.compareTo((BinaryKind) o);
			return UnaryKind.class.getName().compareTo(o.getClass().getName());
		}
	}

	public enum Axis {
		ROWS,
		COLS,
		BOTH,
		TOP
	}
}
