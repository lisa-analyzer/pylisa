package it.unive.pylisa.analysis.dataframes;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Predicate;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.value.ValueLattice;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.HeapAllocationSite;
import it.unive.lisa.lattices.heap.allocations.StackAllocationSite;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.representation.ObjectRepresentation;
import it.unive.lisa.util.representation.SetRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.edge.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

/**
 * The lattice structure of the dataframe-graph abstract domain. The domain
 * operations (i.e. the evaluation of expressions) live in
 * {@link DataframeGraphValueDomain}.
 */
public class DataframeGraphDomain implements ValueLattice<DataframeGraphDomain> {

	public final ValueEnvironment<ConstantPropagation> constants;

	public final ConstantPropagation constStack;

	public final DataframeForest graph;

	public final CollectingMapLattice<Identifier, NodeId> pointers;

	public final CollectingMapLattice<NodeId, DataframeOperation> operations;

	public DataframeGraphDomain() {
		this.constants = new ValueEnvironment<>(new ConstantPropagation()).top();
		this.graph = new DataframeForest(true).top();
		this.pointers = new CollectingMapLattice<Identifier, NodeId>(new SetLattice<>()).top();
		this.operations = new CollectingMapLattice<NodeId, DataframeOperation>(new SetLattice<>()).top();
		this.constStack = this.constants.lattice.top();
	}

	public DataframeForest getGraph() {
		return graph;
	}

	public ValueEnvironment<ConstantPropagation> getConstants() {
		return constants;
	}

	public CollectingMapLattice<Identifier, NodeId> getPointers() {
		return pointers;
	}

	public CollectingMapLattice<NodeId, DataframeOperation> getOperations() {
		return operations;
	}

	DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> pointers,
			CollectingMapLattice<NodeId, DataframeOperation> operations) {
		this(constants, constants.lattice.bottom(), graph, pointers, operations);
	}

	DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			ConstantPropagation constStack,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> pointers,
			CollectingMapLattice<NodeId, DataframeOperation> operations) {
		super();
		this.constants = constants;
		this.constStack = constStack;
		this.graph = graph;
		this.pointers = pointers;

		// cleanup unreachable nodes
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(operations.getMap());
		if (map != null && !map.isEmpty()) {
			Set<NodeId> nodes = new HashSet<>(operations.getKeys());
			for (SetLattice<NodeId> used : this.pointers.getValues())
				used.forEach(nodes::remove);
			pointers.lattice.forEach(nodes::remove);
			nodes.forEach(map::remove);
			this.operations = new CollectingMapLattice<>(operations.lattice, map);
		} else
			this.operations = operations;

		// FIXME temporary sanity check
//		SetLattice<DataframeOperation> pointed = resolvePointers(this);
//		for (DataframeOperation op : pointed)
//			if (!graph.containsNode(op))
//				throw new IllegalStateException();
	}

	@Override
	public DataframeGraphDomain forgetIdentifier(
			Identifier id,
			ProgramPoint pp)
			throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> pointers = this.pointers.lift(i -> id.equals(i) ? null : i, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifier(id, pp),
				graph,
				pointers,
				operations.lift(i -> reverseSearch(i, pointers) ? i : null, e -> e));
	}

	private boolean reverseSearch(
			NodeId id,
			CollectingMapLattice<Identifier, NodeId> map) {
		for (Entry<Identifier, SetLattice<NodeId>> entry : map)
			if (entry.getValue().contains(id))
				return true;

		return false;
	}

	@Override
	public DataframeGraphDomain forgetIdentifiersIf(
			Predicate<Identifier> test,
			ProgramPoint pp)
			throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> pointers = this.pointers.lift(id -> test.test(id) ? null : id, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifiersIf(test, pp),
				graph,
				pointers,
				operations.lift(i -> reverseSearch(i, pointers) ? i : null, e -> e));
	}

	@Override
	public DataframeGraphDomain forgetIdentifiers(
			Iterable<Identifier> ids,
			ProgramPoint pp)
			throws SemanticException {
		Set<Identifier> toForget = new HashSet<>();
		ids.forEach(toForget::add);
		return forgetIdentifiersIf(toForget::contains, pp);
	}

	@Override
	public DataframeGraphDomain store(
			Identifier target,
			Identifier source)
			throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> newPointers = pointers;
		if (!pointers.isTop() && !pointers.isBottom() && pointers.getKeys().contains(source))
			newPointers = pointers.putState(target, pointers.getState(source));
		return new DataframeGraphDomain(
				constants.store(target, source),
				graph,
				newPointers,
				operations);
	}

	@Override
	public DataframeGraphDomain pushScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.pushScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.pushScope(token), e -> e), operations);
					 */
	}

	@Override
	public DataframeGraphDomain popScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.popScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.popScope(token), e -> e), operations);
					 */
	}

	@Override
	public StructuredRepresentation representation() {
		return new ObjectRepresentation(Map.of(
				"constants", constants.representation(),
				"constants-stack", constStack.representation(),
				"pointers", pointers.representation(StringRepresentation::new),
				"pointers-stack", new SetRepresentation(pointers.lattice.elements(), StringRepresentation::new),
				"operations", operations.representation(StringRepresentation::new),
				"graph", graph.representation()));
	}

	@Override
	public DataframeGraphDomain lub(
			DataframeGraphDomain other)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.lub(other.constants),
				constStack.lub(other.constStack),
				graph.lub(other.graph),
				pointers.lub(other.pointers),
				operations.lub(other.operations));
	}

	@Override
	public DataframeGraphDomain widening(
			DataframeGraphDomain other)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.widening(other.constants),
				constStack.widening(other.constStack),
				graph.widening(other.graph),
				pointers.widening(other.pointers),
				operations.widening(other.operations));
	}

	@Override
	public boolean lessOrEqual(
			DataframeGraphDomain other)
			throws SemanticException {
		return constants.lessOrEqual(other.constants)
				&& constStack.lessOrEqual(other.constStack)
				&& graph.lessOrEqual(other.graph)
				&& pointers.lessOrEqual(other.pointers)
				// functional lattice does not check the partial order over the
				// inner lattice instance
				&& pointers.lattice.lessOrEqual(other.pointers.lattice)
				&& operations.lessOrEqual(other.operations);
	}

	@Override
	public DataframeGraphDomain top() {
		return new DataframeGraphDomain(
				constants.top(),
				constStack.top(),
				graph.top(),
				pointers.top(),
				operations.top());
	}

	@Override
	public boolean isTop() {
		return constants.isTop()
				&& constStack.isTop()
				&& graph.isTop()
				&& pointers.isTop()
				&& operations.isTop();
	}

	@Override
	public DataframeGraphDomain bottom() {
		return new DataframeGraphDomain(
				constants.bottom(),
				constStack.bottom(),
				graph.bottom(),
				pointers.bottom(),
				operations.bottom());
	}

	@Override
	public boolean isBottom() {
		return constants.isBottom()
				&& constStack.isBottom()
				&& graph.isBottom()
				&& pointers.isBottom()
				&& operations.isBottom();
	}

	public static AllocationSite stripFields(
			AllocationSite as) {
		if (as.getField() != null)
			// we remove the name of the field using only location name
			if (as instanceof HeapAllocationSite)
				as = new HeapAllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(),
						as.getCodeLocation());
			else
				as = new StackAllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(),
						as.getCodeLocation());
		return as;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constants == null) ? 0 : constants.hashCode());
		result = prime * result + ((constStack == null) ? 0 : constStack.hashCode());
		result = prime * result + ((graph == null) ? 0 : graph.hashCode());
		result = prime * result + ((operations == null) ? 0 : operations.hashCode());
		result = prime * result + ((pointers == null) ? 0 : pointers.hashCode());
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
		DataframeGraphDomain other = (DataframeGraphDomain) obj;
		if (constants == null) {
			if (other.constants != null)
				return false;
		} else if (!constants.equals(other.constants))
			return false;
		if (constStack == null) {
			if (other.constStack != null)
				return false;
		} else if (!constStack.equals(other.constStack))
			return false;
		if (graph == null) {
			if (other.graph != null)
				return false;
		} else if (!graph.equals(other.graph))
			return false;
		if (operations == null) {
			if (other.operations != null)
				return false;
		} else if (!operations.equals(other.operations))
			return false;
		if (pointers == null) {
			if (other.pointers != null)
				return false;
		} else if (!pointers.equals(other.pointers))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	public DataframeForest close() {
		DataframeOperation close = new CloseOperation();

		DataframeForest result = new DataframeForest(graph);
		result.addNode(close);

		Collection<DataframeOperation> exits = new HashSet<>();
		for (SetLattice<NodeId> variable : pointers.getValues())
			for (NodeId id : variable)
				exits.addAll(operations.getState(id).elements());

		for (DataframeOperation op : exits)
			result.addEdge(new SimpleEdge(op, close));

		return result;
	}

	public static class CloseOperation extends DataframeOperation {

		public CloseOperation() {
			super(SyntheticLocation.INSTANCE, -3);
		}

		@Override
		public String toString() {
			return "exit";
		}

		@Override
		protected DataframeOperation lubSameOperation(
				DataframeOperation other)
				throws SemanticException {
			return this;
		}

		@Override
		protected boolean lessOrEqualSameOperation(
				DataframeOperation other)
				throws SemanticException {
			return false;
		}

		@Override
		protected int compareToSameOperation(
				DataframeOperation o) {
			return 0;
		}

		@Override
		protected DataframeOperation wideningSameOperation(
				DataframeOperation other)
				throws SemanticException {
			return this;
		}
	}

	public Map<Identifier, DataframeForest> partitionByVarialbe() {
		Map<Identifier, DataframeForest> result = new HashMap<>();

		for (Entry<Identifier, SetLattice<NodeId>> entry : pointers.getMap().entrySet())
			if (!entry.getValue().isTop() && !entry.getValue().isBottom()) {
				DataframeForest accumulator = null;
				for (NodeId id : entry.getValue())
					for (DataframeOperation op : operations.getState(id)) {
						DataframeForest sub = graph.bDFS(op,
								o -> false,
								edge -> true);
						if (accumulator == null)
							accumulator = sub;
						else
							accumulator = accumulator.union(sub);
					}
				result.put(entry.getKey(), accumulator);
			}

		return result;
	}

	@Override
	public boolean knowsIdentifier(
			Identifier id) {
		return constants.knowsIdentifier(id) || pointers.getKeys().contains(id);
	}
}
