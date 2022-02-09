package it.unive.pylisa.analysis.dataframes.structure;

import java.util.HashSet;
import java.util.Set;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.libraries.pandas.types.PandasType;

public class SingleDataframe extends BaseNonRelationalValueDomain<SingleDataframe> {

	private final String file;
	private final ColumnSet names;
	private final ColumnSet possibleNames;
	private final ColumnMapping mapping;
	private final Interval rows;

	public SingleDataframe() {
		this((String) null);
	}

	public SingleDataframe(String file) {
		this(file, new ColumnSet(), new ColumnSet(), new ColumnMapping(), new Interval().top());
	}

	public SingleDataframe(String file, ColumnSet names, ColumnSet possibleNames, ColumnMapping mapping,
			Interval rows) {
		this.file = file;
		this.names = names;
		this.possibleNames = possibleNames;
		this.mapping = mapping;
		this.rows = rows;
	}

	@Override
	public SingleDataframe top() {
		return new SingleDataframe(
				null,
				names.top(),
				possibleNames.top(),
				mapping.top(),
				rows.top());
	}

	@Override
	public boolean isTop() {
		return file == null
				&& names.isTop()
				&& possibleNames.isTop()
				&& mapping.isTop()
				&& rows.isTop();
	}

	@Override
	public SingleDataframe bottom() {
		return new SingleDataframe(
				null,
				names.bottom(),
				possibleNames.bottom(),
				mapping.bottom(),
				rows.bottom());
	}

	@Override
	public boolean isBottom() {
		return file == null
				&& names.isBottom()
				&& possibleNames.isBottom()
				&& mapping.isBottom()
				&& rows.isBottom();
	}

	@Override
	protected SingleDataframe lubAux(SingleDataframe other) throws SemanticException {
		if (!file.equals(other.file))
			return top();
		return new SingleDataframe(
				file,
				names.lub(other.names),
				possibleNames.lub(other.possibleNames),
				mapping.lub(other.mapping),
				rows.lub(other.rows));
	}

	@Override
	protected SingleDataframe wideningAux(SingleDataframe other) throws SemanticException {
		if (!file.equals(other.file))
			return top();
		return new SingleDataframe(
				file,
				names.widening(other.names),
				possibleNames.widening(other.possibleNames),
				mapping.widening(other.mapping),
				rows.widening(other.rows));
	}

	@Override
	protected boolean lessOrEqualAux(SingleDataframe other) throws SemanticException {
		return file.equals(other.file)
				&& names.lessOrEqual(other.names)
				&& possibleNames.lessOrEqual(other.possibleNames)
				&& mapping.lessOrEqual(other.mapping)
				&& rows.lessOrEqual(other.rows);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((file == null) ? 0 : file.hashCode());
		result = prime * result + ((mapping == null) ? 0 : mapping.hashCode());
		result = prime * result + ((names == null) ? 0 : names.hashCode());
		result = prime * result + ((possibleNames == null) ? 0 : possibleNames.hashCode());
		result = prime * result + ((rows == null) ? 0 : rows.hashCode());
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
		SingleDataframe other = (SingleDataframe) obj;
		if (file == null) {
			if (other.file != null)
				return false;
		} else if (!file.equals(other.file))
			return false;
		if (mapping == null) {
			if (other.mapping != null)
				return false;
		} else if (!mapping.equals(other.mapping))
			return false;
		if (names == null) {
			if (other.names != null)
				return false;
		} else if (!names.equals(other.names))
			return false;
		if (possibleNames == null) {
			if (other.possibleNames != null)
				return false;
		} else if (!possibleNames.equals(other.possibleNames))
			return false;
		if (rows == null) {
			if (other.rows != null)
				return false;
		} else if (!rows.equals(rows))
			return false;
		return true;
	}

	@Override
	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;

		if (isBottom())
			return Lattice.BOTTOM_REPR;

		return new DFRepresentation();
	}

	private class DFRepresentation extends DomainRepresentation {

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + getEnclosingInstance().hashCode();
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
			DFRepresentation other = (DFRepresentation) obj;
			if (!getEnclosingInstance().equals(other.getEnclosingInstance()))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return "file: " + file + ", cols: " + names.representation() + ", mapping: " + mapping.representation()
					+ ", possible cols: " + possibleNames.representation() + ", rows: " + rows.representation();
		}

		private SingleDataframe getEnclosingInstance() {
			return SingleDataframe.this;
		}
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return canProcess(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes().anyMatch(PandasType.class::isInstance)
				: expression.getStaticType() instanceof PandasType || expression.getStaticType().isUntyped();
	}

	public SingleDataframe addColumn(String name, boolean definite) {
		Set<String> copy;
		if (definite)
			copy = new HashSet<>(names.elements() == null ? new HashSet<>() : names.elements());
		else
			copy = new HashSet<>(possibleNames.elements() == null ? new HashSet<>() : possibleNames.elements());

		copy.add(name);

		if (definite)
			return new SingleDataframe(file, new ColumnSet(copy), possibleNames, mapping, rows);
		else
			return new SingleDataframe(file, names, new ColumnSet(copy), mapping, rows);
	}

	public SingleDataframe accessRows(int low, int high) throws SemanticException {
		Interval lub = rows.lub(new Interval(low, high));
		return new SingleDataframe(file, names, possibleNames, mapping, lub);
	}
}
