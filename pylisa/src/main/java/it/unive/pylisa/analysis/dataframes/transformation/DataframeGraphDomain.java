package it.unive.pylisa.analysis.dataframes.transformation;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.lisa.util.collections.workset.WorkingSet;
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

	public DataframeGraphDomain(DataframeGraphDomain source, DataframeOperation transformation)
			throws SemanticException {
		this(append(source.transformations, transformation), false);
	}

	public DataframeGraphDomain(DataframeGraph source, DataframeOperation transformation)
			throws SemanticException {
		this(append(source, transformation), false);
	}

	public static DataframeGraph append(DataframeGraph source, DataframeOperation transformation)
			throws SemanticException {
		if (source.getNodesCount() == 0) {
			DataframeGraph graph = new DataframeGraph();
			graph.addNode(transformation);
			return graph;
		}

		DataframeGraph copy = new DataframeGraph(source);
		copy.addNode(transformation);
		copy.addEdge(new SimpleEdge(source.getLeaf(), transformation));
		return copy;
	}

	public DataframeGraphDomain(DataframeGraph transformations) {
		this(transformations, false);
	}

	private DataframeGraphDomain(DataframeGraph transformations, boolean isTop) {
		this.transformations = transformations;
		this.isTop = isTop;
	}

	public DataframeGraph getTransformations() {
		return transformations;
	}

	@Override
	protected DataframeGraphDomain lubAux(DataframeGraphDomain other) throws SemanticException {
		DataframeGraph tm = transformations;
		DataframeGraph om = other.transformations;
		if (!check(tm, om))
			return top();

		// this is a sequential list: we can just visit it

		WorkingSet<DataframeOperation> tws = FIFOWorkingSet.mk();
		WorkingSet<DataframeOperation> ows = FIFOWorkingSet.mk();

		tws.push(tm.getEntries().iterator().next());
		ows.push(om.getEntries().iterator().next());

		DataframeOperation t, o, r, pr = null;
		DataframeGraph rm = new DataframeGraph();
		while (!tws.isEmpty() || !ows.isEmpty()) {
			t = tws.isEmpty() ? DataframeOperation.BOTTOM : tws.peek();
			o = ows.isEmpty() ? DataframeOperation.BOTTOM : ows.peek();
			r = t.lub(o);

			rm.addNode(r);
			if (pr != null)
				rm.addEdge(new SimpleEdge(pr, r));

			pr = r;
			if (!tws.isEmpty()) {
				tws.pop();
				if (!tm.followersOf(t).isEmpty())
					tws.push(tm.followersOf(t).iterator().next());
			}

			if (!ows.isEmpty()) {
				ows.pop();
				if (!om.followersOf(o).isEmpty())
					ows.push(om.followersOf(o).iterator().next());
			}
		}

		return new DataframeGraphDomain(new DataframeGraph(rm), false);
	}

	@Override
	protected DataframeGraphDomain wideningAux(DataframeGraphDomain other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeGraphDomain other) throws SemanticException {
		DataframeGraph tm = transformations;
		DataframeGraph om = other.transformations;
		if (!check(tm, om))
			return false;

		// this is a sequential list: we can just visit it

		WorkingSet<DataframeOperation> tws = FIFOWorkingSet.mk();
		WorkingSet<DataframeOperation> ows = FIFOWorkingSet.mk();

		tws.push(tm.getEntries().iterator().next());
		ows.push(om.getEntries().iterator().next());

		DataframeOperation t, o;
		while (!tws.isEmpty() || !ows.isEmpty()) {
			t = tws.isEmpty() ? DataframeOperation.BOTTOM : tws.peek();
			o = ows.isEmpty() ? DataframeOperation.BOTTOM : ows.peek();
			if (!t.lessOrEqual(o))
				return false;

			if (!tws.isEmpty()) {
				tws.pop();
				if (!tm.followersOf(t).isEmpty())
					tws.push(tm.followersOf(t).iterator().next());
			}

			if (!ows.isEmpty()) {
				ows.pop();
				if (!om.followersOf(o).isEmpty())
					ows.push(om.followersOf(o).iterator().next());
			}
		}

		return true;
	}

	private boolean check(DataframeGraph tm, DataframeGraph om) {
		if (tm.getEntries().size() != 1 || om.getEntries().size() != 1)
			return false;
		if (tm.getExits().size() != 1 || om.getExits().size() != 1)
			return false;

		for (DataframeOperation op : tm.getNodes())
			if (tm.getOutgoingEdges(op).size() > 1)
				return false;

		for (DataframeOperation op : om.getNodes())
			if (om.getOutgoingEdges(op).size() > 1)
				return false;

		return true;
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
				? expression.getRuntimeTypes().anyMatch(t -> PandasType.isPandasType(t, false))
				: PandasType.isPandasType(expression.getStaticType(), false) || expression.getStaticType().isUntyped();
	}

	@Override
	public String toString() {
		return representation().toString();
	}
}
