package it.unive.pylisa.checks;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;

import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.CFGWithAnalysisResults;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.AllocationSites;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.outputs.serializableGraph.SerializableString;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.analysis.dataframes.graph.CollectingMapLattice;
import it.unive.pylisa.analysis.dataframes.graph.DataframeForest;
import it.unive.pylisa.analysis.dataframes.graph.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.graph.NodeId;
import it.unive.pylisa.analysis.dataframes.graph.SetLattice;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;

public class DataframeDumper implements SemanticCheck<
		SimpleAbstractState<
				PointBasedHeap,
				DataframeGraphDomain,
				TypeEnvironment<InferredTypes>>,
		PointBasedHeap,
		DataframeGraphDomain,
		TypeEnvironment<InferredTypes>> {

	private final FileManager fileManager;

	public DataframeDumper(LiSAConfiguration conf) {
		this.fileManager = new FileManager(conf.getWorkdir());
	}

	@Override
	public void beforeExecution(CheckToolWithAnalysisResults<
			SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool) {
	}

	@Override
	public void afterExecution(CheckToolWithAnalysisResults<
			SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool) {
	}

	@Override
	public boolean visitCompilationUnit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
					PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool,
			CompilationUnit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
					PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool,
			Unit unit, Global global, boolean instance) {
	}

	@Override
	public boolean visit(CheckToolWithAnalysisResults<
			SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool, CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
					PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool,
			CFG graph, Statement node) {
		if (!graph.getDescriptor().getName().equals("main"))
			return true;
		
		if (node.stopsExecution()) {
			Collection<
					CFGWithAnalysisResults<
							SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
									TypeEnvironment<InferredTypes>>,
							PointBasedHeap, DataframeGraphDomain,
							TypeEnvironment<InferredTypes>>> results = tool.getResultOf(graph);

			for (CFGWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
					PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> result : results) {

				AnalysisState<
						SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
								TypeEnvironment<InferredTypes>>,
						PointBasedHeap, DataframeGraphDomain,
						TypeEnvironment<InferredTypes>> post = result.getAnalysisStateAfter(node);

				String filename = result.getDescriptor().getFullSignatureWithParNames();
				if (result.getId() != null)
					filename += "_" + result.getId().hashCode();

				PointBasedHeap heap = post.getDomainInstance(PointBasedHeap.class);
				DataframeGraphDomain dom = post.getDomainInstance(DataframeGraphDomain.class);
				DataframeForest forest = dom.getGraph();
				CollectingMapLattice<Identifier, NodeId> pointers = dom.getPointers();
				CollectingMapLattice<NodeId, DataframeOperation> operations = dom.getOperations();

				Map<DataframeOperation, Set<Identifier>> refs = new HashMap<>();
				for (Entry<Identifier, SetLattice<NodeId>> pointer : pointers) {
					Set<Identifier> ids = reverse(heap, pointer.getKey());
					if (ids.isEmpty())
						continue;

					for (NodeId n : pointer.getValue())
						operations.getState(n).elements()
								.forEach(op -> refs.computeIfAbsent(op, o -> new HashSet<>()).addAll(ids));
				}

				try {
					fileManager.mkDotFile(filename + "@" + node.getOffset(),
							writer -> forest.toSerializableGraph(op -> label(op, refs)).toDot().dump(writer));
				} catch (IOException e) {
					throw new RuntimeException(e);
				}
			}
		}

		return true;
	}

	private SerializableValue label(DataframeOperation op, Map<DataframeOperation, Set<Identifier>> refs) {
		String extra = refs.containsKey(op)
				? "\nPointed by: " + refs.get(op).stream().map(id -> id.toString()).sorted()
						.reduce("", (res, s) -> res + s + "\n").trim()
				: "";
		return new SerializableString(new TreeMap<>(), "at: " + op.getWhere().getCodeLocation() + extra);
	}

	@SuppressWarnings("unchecked")
	private Set<Identifier> reverse(PointBasedHeap heap, Identifier key) {
		if (!(key instanceof AllocationSite))
			return Collections.emptySet();

		Set<Identifier> result = new HashSet<>();
		try {
			Field envField = PointBasedHeap.class.getDeclaredField("heapEnv");
			envField.setAccessible(true);
			HeapEnvironment<AllocationSites> env = (HeapEnvironment<AllocationSites>) envField.get(heap);
			for (Entry<Identifier, AllocationSites> v : env)
				if (v.getValue().elements().stream()
						.anyMatch(site -> DataframeGraphDomain.stripFields(site).equals((AllocationSite) key)))
					result.add(v.getKey());
			return result;
		} catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
		}
		return Collections.emptySet();
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>,
					PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>> tool,
			CFG graph, Edge edge) {
		return true;
	}
}
