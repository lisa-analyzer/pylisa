package it.unive.pylisa.analysis.dataframes.transformation;

import java.util.Collection;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.libraries.pandas.types.PandasType;

public class DataframeGraphDomain extends BaseLattice<DataframeGraphDomain> {

	private static final DataframeGraph NO_TRANSFORMATIONS = new DataframeGraph();

	private static final DataframeGraphDomain BOTTOM = new DataframeGraphDomain(false);
	private static final DataframeGraphDomain TOP = new DataframeGraphDomain(true);

	private final DataframeGraph transformations;
	private final boolean isTop;

	public DataframeGraphDomain() {
		this(NO_TRANSFORMATIONS, true);
	}

	private DataframeGraphDomain(boolean isTop) {
		this(NO_TRANSFORMATIONS, isTop);
	}

	public DataframeGraphDomain(DataframeOperation transformation) throws SemanticException {
		this(append(NO_TRANSFORMATIONS, transformation), false);
	}

	public DataframeGraphDomain(DataframeGraphDomain source, DataframeOperation transformation) throws SemanticException {
		this(append(source.transformations, transformation), false);
	}

	private static DataframeGraph append(DataframeGraph source, DataframeOperation transformation)
			throws SemanticException {
		if (source.getNodesCount() == 0) {
			DataframeGraph graph = new DataframeGraph();
			graph.addNode(transformation, true);
			return graph;
		}

		Collection<DataframeOperation> exits = source.getAdjacencyMatrix().getExits();
		if (exits.size() != 1)
			throw new SemanticException("Appending an operation to a graph with more than one leaf");
		DataframeGraph copy = new DataframeGraph(source);
		copy.addNode(transformation);
		copy.addEdge(new SimpleEdge(exits.iterator().next(), transformation));
		return copy;
	}

	private DataframeGraphDomain(DataframeGraph transformations, boolean isTop) {
		this.transformations = transformations;
		this.isTop = isTop;
	}

	@Override
	protected DataframeGraphDomain lubAux(DataframeGraphDomain other) throws SemanticException {
		return equals(other) ? this : top();
	}

	@Override
	protected DataframeGraphDomain wideningAux(DataframeGraphDomain other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeGraphDomain other) throws SemanticException {
		return equals(other);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (isTop ? 1231 : 1237);
		result = prime * result + ((transformations == null) ? 0 : transformations.hashCode());
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
		DataframeGraphDomain other = (DataframeGraphDomain) obj;
		if (isTop != other.isTop)
			return false;
		if (transformations == null) {
			if (other.transformations != null)
				return false;
		} else if (!transformations.equals(other.transformations))
			return false;
		return true;
	}

	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		return new StringRepresentation(transformations);
	}

	@Override
	public DataframeGraphDomain top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return transformations.getNodesCount() == 0 && isTop;
	}

	@Override
	public DataframeGraphDomain bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return transformations.getNodesCount() == 0 && !isTop;
	}

	public static boolean tracks(Identifier id) {
		return processes(id);
	}

	public static boolean processes(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes().anyMatch(PandasType.class::isInstance)
				: expression.getStaticType() instanceof PandasType || expression.getStaticType().isUntyped();
	}

	@Override
	public String toString() {
		return representation().toString();
	}
}
