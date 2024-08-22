package it.unive.pylisa.checks;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import it.unive.lisa.AnalysisExecutionException;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.FunctionalLattice;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.lisa.util.datastructures.graph.algorithms.Fixpoint;
import it.unive.lisa.util.datastructures.graph.algorithms.Fixpoint.FixpointImplementation;
import it.unive.lisa.util.datastructures.graph.algorithms.FixpointException;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain.CloseOperation;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.edge.AssignEdge;
import it.unive.pylisa.analysis.dataframes.edge.ConsumeEdge;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.operations.Assign;
import it.unive.pylisa.analysis.dataframes.operations.BottomOperation;
import it.unive.pylisa.analysis.dataframes.operations.Concat;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.operations.GetAxis;
import it.unive.pylisa.analysis.dataframes.operations.Init;
import it.unive.pylisa.analysis.dataframes.operations.Iteration;
import it.unive.pylisa.analysis.dataframes.operations.Project;
import it.unive.pylisa.analysis.dataframes.operations.Read;
import it.unive.pylisa.analysis.dataframes.operations.Reshape;
import it.unive.pylisa.analysis.dataframes.operations.Transform;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.BooleanSelection;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.BinaryTransformKind;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.UnaryTransformKind;

public class DataframeStructureConstructor
		implements
		SemanticCheck<
				SimpleAbstractState<
						PointBasedHeap,
						DataframeGraphDomain,
						TypeEnvironment<InferredTypes>>> {

	@Override
	public void beforeExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool) {
	}

	@Override
	public void afterExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool) {
	}

	@Override
	public boolean visitUnit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			Unit unit,
			Global global,
			boolean instance) {
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Edge edge) {
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Statement node) {
		if (!graph.getDescriptor().getName().equals(PyFrontend.INSTRUMENTED_MAIN_FUNCTION_NAME))
			return true;

		if (node.stopsExecution()) {
			Collection<AnalyzedCFG<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
							TypeEnvironment<InferredTypes>>>> results = tool.getResultOf(graph);

			for (AnalyzedCFG<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
							TypeEnvironment<InferredTypes>>> result : results)
				try {
					AnalysisState<
							SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
									TypeEnvironment<InferredTypes>>> post = result.getAnalysisStateAfter(node);

					DataframeGraphDomain dom = post.getState().getDomainInstance(DataframeGraphDomain.class);
					DataframeForest forest = dom.close();
					Collection<DataframeOperation> exits = forest.getNodeList().getExits();
					if (exits.size() != 1)
						throw new IllegalStateException("Close operation failed");
					DataframeOperation exit = exits.iterator().next();

					ColumnsDomain columnsDomain;
					try {
						columnsDomain = process(forest, exit);
					} catch (FixpointException e) {
						throw new RuntimeException("Processing failed", e);
					}

					for (Entry<Names, Columns> entry : columnsDomain) {
						Names sources = entry.getKey();
						Columns columns = entry.getValue();
						if (!columns.accessedBeforeAssigned.isEmpty())
							tool.warn(sources + " columns accessed before being assigned: "
									+ columns.accessedBeforeAssigned);
						if (!columns.accessedAfterRemoved.isEmpty())
							tool.warn(
									sources + " columns accessed after being removed: " + columns.accessedAfterRemoved);
						if (!columns.accessed.isEmpty())
							tool.warn(sources + " columns accessed: " + columns.accessed);
						if (!columns.assigned.isEmpty())
							tool.warn(sources + " columns assigned: " + columns.assigned);
						if (!columns.removed.isEmpty())
							tool.warn(sources + " columns removed: " + columns.removed);
					}
				} catch (SemanticException e) {
					throw new AnalysisExecutionException(e);
				}
		}

		return true;
	}

	private ColumnsDomain process(
			DataframeForest graph,
			DataframeOperation exit)
			throws FixpointException {
		Fixpoint<DataframeForest, DataframeOperation, DataframeEdge, ColumnsDomain> fix = new Fixpoint<>(graph, false);
		ColumnsDomain beginning = new ColumnsDomain(Columns.TOP).bottom();

		Map<DataframeOperation, ColumnsDomain> entrypoints = new HashMap<>();
		for (DataframeOperation entry : graph.getNodeList().getEntries())
			entrypoints.put(entry, beginning);

		Map<DataframeOperation, ColumnsDomain> fixpoint = fix.fixpoint(entrypoints, FIFOWorkingSet.mk(),
				new FixpointImplementation<DataframeOperation, DataframeEdge,
						DataframeStructureConstructor.ColumnsDomain>() {

					@Override
					public ColumnsDomain union(
							DataframeOperation node,
							ColumnsDomain left,
							ColumnsDomain right)
							throws Exception {
						return operation(node, left, right);
					}

					@Override
					public ColumnsDomain traverse(
							DataframeEdge edge,
							ColumnsDomain entrystate)
							throws Exception {
						return entrystate;
					}

					@Override
					public boolean equality(
							DataframeOperation node,
							ColumnsDomain approx,
							ColumnsDomain old)
							throws Exception {
						return approx.lessOrEqual(old);
					}

					@Override
					public ColumnsDomain semantics(
							DataframeOperation node,
							ColumnsDomain entrystate)
							throws Exception {
						Names sources = extractSources(node, graph);

						if (node instanceof Assign<?, ?>)
							return entrystate.assign(sources,
									((Assign<?, ?>) node).getSelection().extractColumnNames());
						else if (node instanceof Project<?, ?>) {
							boolean allConsume = true;
							Project<?, ?> proj = (Project<?, ?>) node;
							for (DataframeEdge edge : graph.getOutgoingEdges(node))
								if (edge.getDestination().equals(exit))
									continue;
								else if (!(edge instanceof ConsumeEdge)) {
									allConsume = false;
									break;
								}
							if (proj.getSelection().getRowSelection() instanceof BooleanSelection<?>)
								// boolean selections are always used to produce
								// the boolean masks, even when they are on the
								// lhs of an assignment
								entrystate = entrystate.access(sources,
										proj.getSelection().getRowSelection().extractColumnNames());

							if (allConsume)
								// will be reported separately as selection of
								// the consumer
								return entrystate;
							return entrystate.access(sources, proj.getSelection().extractColumnNames());
						} else if (node instanceof Transform<?, ?>) {
							Transform<?, ?> transform = (Transform<?, ?>) node;
							if (transform.getType() == BinaryTransformKind.ASSIGN)
								return entrystate.assign(sources, transform.getSelection().extractColumnNames());
							else if (transform.getType() == UnaryTransformKind.DROP_COLS)
								return entrystate.remove(sources, transform.getSelection().extractColumnNames());
							else
								return entrystate.access(sources, transform.getSelection().extractColumnNames());
						} else if (node instanceof Reshape<?, ?>)
							return entrystate.define(sources);
						else if (node instanceof Read || node instanceof Concat)
							return entrystate.define(sources);
						else if (node instanceof Init || node instanceof BottomOperation
								|| node instanceof CloseOperation || node instanceof Iteration
								|| node instanceof GetAxis)
							return entrystate;
						else
							return entrystate.top();
					}

					private Names extractSources(
							DataframeOperation node,
							DataframeForest graph) {
						DataframeForest cut = graph.bDFS(node,
								op -> op instanceof Reshape<?, ?>,
								edge -> !(edge instanceof AssignEdge));
						Set<String> names = new HashSet<>();
						for (DataframeOperation op : cut.getNodeList().getEntries())
							if (op instanceof Read
									&& !((Read) op).getFile().isTop()
									&& !((Read) op).getFile().isBottom())
								names.add(((Read) op).getFile().as(String.class));
							else
								names.add(op.toString());
						return new Names(names);
					}

					@Override
					public ColumnsDomain operation(
							DataframeOperation node,
							ColumnsDomain approx,
							ColumnsDomain old)
							throws Exception {
						return approx.lub(old);
					}
				});

		return fixpoint.get(exit);
	}

	private static class ColumnsDomain extends FunctionalLattice<ColumnsDomain, Names, Columns> {

		public ColumnsDomain(
				Columns lattice,
				Map<Names, Columns> function) {
			super(lattice, function);
		}

		public ColumnsDomain define(
				Names sources) {
			return putState(sources, Columns.BOTTOM);
		}

		public ColumnsDomain(
				Columns lattice) {
			super(lattice);
		}

		public ColumnsDomain assign(
				Names key,
				Names names)
				throws SemanticException {
			return putState(key, getState(key).assign(names));
		}

		public ColumnsDomain remove(
				Names key,
				Names names)
				throws SemanticException {
			return putState(key, getState(key).remove(names));
		}

		public ColumnsDomain access(
				Names key,
				Names names)
				throws SemanticException {
			return putState(key, getState(key).access(names));
		}

		@Override
		public ColumnsDomain top() {
			return new ColumnsDomain(lattice.top(), null);
		}

		@Override
		public ColumnsDomain bottom() {
			return new ColumnsDomain(lattice.bottom(), null);
		}

		@Override
		public ColumnsDomain mk(
				Columns lattice,
				Map<Names, Columns> function) {
			return new ColumnsDomain(lattice, function);
		}

		@Override
		public Columns stateOfUnknown(
				Names key) {
			return lattice.bottom();
		}
	}

	private static class Columns implements BaseLattice<Columns> {

		private static final Columns TOP = new Columns(Names.TOP, Names.TOP, Names.TOP, Names.TOP, Names.TOP);
		private static final Columns BOTTOM = new Columns(Names.BOTTOM, Names.BOTTOM, Names.BOTTOM, Names.BOTTOM,
				Names.BOTTOM);

		private Names accessedBeforeAssigned;
		private Names accessedAfterRemoved;
		private Names accessed;
		private Names assigned;
		private Names removed;

		public Columns(
				Names accessed,
				Names assigned,
				Names removed,
				Names accessedBeforeAssigned,
				Names accessedAfterRemoved) {
			this.accessedBeforeAssigned = accessedBeforeAssigned;
			this.accessedAfterRemoved = accessedAfterRemoved;
			this.accessed = accessed;
			this.assigned = assigned;
			this.removed = removed;
		}

		public Columns assign(
				Names names)
				throws SemanticException {
			Names assigned = this.assigned.lub(names);
			return new Columns(accessed, assigned, removed, accessedBeforeAssigned, accessedAfterRemoved);
		}

		public Columns remove(
				Names names)
				throws SemanticException {
			Names removed = this.removed.lub(names);
			return new Columns(accessed, removed, removed, accessedBeforeAssigned, accessedAfterRemoved);
		}

		public Columns access(
				Names names)
				throws SemanticException {
			Names accessedAndRemoved = this.removed.intersection(names);
			Names accessedAndAssigned = this.assigned.intersection(names);

			Names accessedTooEarly = names.difference(accessedAndAssigned).difference(accessedAndRemoved);

			Names accessedBeforeAssigned = this.accessedBeforeAssigned.lub(accessedTooEarly);
			Names accessedAfterRemoved = this.accessedAfterRemoved.lub(accessedAndRemoved);
			Names accessed = this.accessed.lub(accessedAndAssigned.difference(accessedAndRemoved));
			return new Columns(accessed, this.assigned, this.removed, accessedBeforeAssigned, accessedAfterRemoved);
		}

		@Override
		public Columns top() {
			return TOP;
		}

		@Override
		public Columns bottom() {
			return BOTTOM;
		}

		@Override
		public Columns lubAux(
				Columns other)
				throws SemanticException {
			Names accessed = this.accessed.lub(other.accessed);
			Names assigned = this.assigned.lub(other.assigned);
			Names removed = this.removed.lub(other.removed);
			Names accessedBeforeAssigned = this.accessedBeforeAssigned
					.lub(other.accessedBeforeAssigned);
			Names accessedAfterRemoved = this.accessedAfterRemoved.lub(other.accessedAfterRemoved);
			return new Columns(accessed, assigned, removed, accessedBeforeAssigned, accessedAfterRemoved);
		}

		@Override
		public Columns wideningAux(
				Columns other)
				throws SemanticException {
			return lubAux(other);
		}

		@Override
		public boolean lessOrEqualAux(
				Columns other)
				throws SemanticException {
			return accessed.lessOrEqual(other.accessed)
					&& assigned.lessOrEqual(other.assigned)
					&& removed.lessOrEqual(other.removed)
					&& accessedBeforeAssigned.lessOrEqual(other.accessedBeforeAssigned)
					&& accessedAfterRemoved.lessOrEqual(other.accessedAfterRemoved);
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + ((accessed == null) ? 0 : accessed.hashCode());
			result = prime * result + ((accessedAfterRemoved == null) ? 0 : accessedAfterRemoved.hashCode());
			result = prime * result + ((accessedBeforeAssigned == null) ? 0 : accessedBeforeAssigned.hashCode());
			result = prime * result + ((assigned == null) ? 0 : assigned.hashCode());
			result = prime * result + ((removed == null) ? 0 : removed.hashCode());
			return result;
		}

		@Override
		public boolean equals(
				Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			Columns other = (Columns) obj;
			if (accessed == null) {
				if (other.accessed != null)
					return false;
			} else if (!accessed.equals(other.accessed))
				return false;
			if (accessedAfterRemoved == null) {
				if (other.accessedAfterRemoved != null)
					return false;
			} else if (!accessedAfterRemoved.equals(other.accessedAfterRemoved))
				return false;
			if (accessedBeforeAssigned == null) {
				if (other.accessedBeforeAssigned != null)
					return false;
			} else if (!accessedBeforeAssigned.equals(other.accessedBeforeAssigned))
				return false;
			if (assigned == null) {
				if (other.assigned != null)
					return false;
			} else if (!assigned.equals(other.assigned))
				return false;
			if (removed == null) {
				if (other.removed != null)
					return false;
			} else if (!removed.equals(other.removed))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return "accessedBeforeAssigned=" + accessedBeforeAssigned + "\n"
					+ "accessedAfterRemoved=" + accessedAfterRemoved + "\n"
					+ "accessed=" + accessed + "\n"
					+ "assigned=" + assigned + "\n"
					+ "removed=" + removed;
		}

		@Override
		public StructuredRepresentation representation() {
			return new StringRepresentation(toString());
		}
	}
}