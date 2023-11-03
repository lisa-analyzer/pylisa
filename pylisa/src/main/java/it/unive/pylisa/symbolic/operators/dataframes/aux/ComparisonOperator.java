package it.unive.pylisa.symbolic.operators.dataframes.aux;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;

public class ComparisonOperator implements BaseLattice<ComparisonOperator>, Comparable<ComparisonOperator> {

	public static enum Operator {
		EQ("=="),
		NE("!="),
		LT("<"),
		LE("<="),
		GT(">"),
		GE(">="),
		TOP(Lattice.TOP_STRING),
		BOTTOM(Lattice.BOTTOM_STRING);

		private final String symbol;

		private Operator(
				String symbol) {
			this.symbol = symbol;
		}

		@Override
		public String toString() {
			return symbol;
		}
	}

	private Operator op;

	public ComparisonOperator(
			Operator op) {
		this.op = op;
	}

	public static final ComparisonOperator EQ = new ComparisonOperator(Operator.EQ);
	public static final ComparisonOperator NEQ = new ComparisonOperator(Operator.NE);
	public static final ComparisonOperator GT = new ComparisonOperator(Operator.GT);
	public static final ComparisonOperator GEQ = new ComparisonOperator(Operator.GE);
	public static final ComparisonOperator LT = new ComparisonOperator(Operator.LT);
	public static final ComparisonOperator LEQ = new ComparisonOperator(Operator.LE);
	public static final ComparisonOperator TOP = new ComparisonOperator(Operator.TOP);
	public static final ComparisonOperator BOT = new ComparisonOperator(Operator.BOTTOM);

	@Override
	public ComparisonOperator top() {
		return new ComparisonOperator(Operator.TOP);
	}

	@Override
	public ComparisonOperator bottom() {
		return new ComparisonOperator(Operator.TOP);
	}

	@Override
	public ComparisonOperator lubAux(
			ComparisonOperator other)
			throws SemanticException {
		if (lessOrEqual(other)) {
			return other;
		}
		return top();
	}

	@Override
	public ComparisonOperator wideningAux(
			ComparisonOperator other)
			throws SemanticException {
		return lub(other);
	}

	@Override
	public boolean lessOrEqualAux(
			ComparisonOperator other)
			throws SemanticException {
		if (this.op == Operator.BOTTOM) {
			return true;
		} else if (this.op == other.op) {
			return true;
		}
		return false;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (obj == null || !(obj instanceof ComparisonOperator))
			return false;
		ComparisonOperator o = (ComparisonOperator) obj;
		if (op != o.op)
			return false;
		return true;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((op == null) ? 0 : op.hashCode());
		return result;
	}

	@Override
	public String toString() {
		return op.toString();
	}

	@Override
	public int compareTo(
			ComparisonOperator other) {
		return op.compareTo(other.op);
	}

}
