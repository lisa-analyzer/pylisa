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
import java.util.function.Function;

import it.unive.lisa.AnalysisExecutionException;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
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
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.util.file.FileManager.WriteAction;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataframes.CollectingMapLattice;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.NodeId;
import it.unive.pylisa.analysis.dataframes.SetLattice;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class DataframeDumper
		implements
		SemanticCheck<
				SimpleAbstractState<
						PointBasedHeap,
						DataframeGraphDomain,
						TypeEnvironment<InferredTypes>>> {

	public DataframeDumper() {
	}

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
			Statement node) {
		if (!graph.getDescriptor().getName().equals(PyFrontend.INSTRUMENTED_MAIN_FUNCTION_NAME))
			return true;

		if (node.stopsExecution()) {
			Collection<
					AnalyzedCFG<
							SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
									TypeEnvironment<InferredTypes>>>> results = tool.getResultOf(graph);

			for (AnalyzedCFG<
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
							TypeEnvironment<InferredTypes>>> result : results)
				try {
					AnalysisState<
							SimpleAbstractState<PointBasedHeap, DataframeGraphDomain,
									TypeEnvironment<InferredTypes>>> post = result.getAnalysisStateAfter(node);
					PointBasedHeap heap = post.getState().getDomainInstance(PointBasedHeap.class);
					DataframeGraphDomain dom = post.getState().getDomainInstance(DataframeGraphDomain.class);
					DataframeForest forest = dom.getGraph();
					Collection<DataframeForest> subgraphs = forest.partitionByRoot();
					Map<Identifier, DataframeForest> vargraphs = dom.partitionByVarialbe();
					CollectingMapLattice<Identifier, NodeId> pointers = dom.getPointers();
					CollectingMapLattice<NodeId, DataframeOperation> operations = dom.getOperations();

					Map<DataframeOperation, Set<Identifier>> refs = new HashMap<>();
					Map<Identifier, Set<Identifier>> reverseMap = new HashMap<>();
					for (Entry<Identifier, SetLattice<NodeId>> pointer : pointers) {
						Set<Identifier> ids = reverse(heap, pointer.getKey());
						if (ids.isEmpty())
							continue;

						reverseMap.put(pointer.getKey(), ids);
						for (NodeId n : pointer.getValue())
							operations.getState(n).elements()
									.forEach(op -> refs.computeIfAbsent(op, o -> new HashSet<>()).addAll(ids));
					}

					Function<DataframeForest,
							WriteAction> jsonFactory = _forest -> writer -> _forest
									.toSerializableGraph((
											f,
											op) -> label(op, refs))
									.dump(writer);
					Function<DataframeForest,
							WriteAction> dotFactory = _forest -> writer -> _forest
									.toSerializableGraph((
											f,
											op) -> label(op, refs))
									.toDot()
									.dump(writer);
					try {
						String name = "forest@" + node.getLocation();
						if (!result.getId().isStartingId())
							name += "_" + result.getId().hashCode();
						tool.getFileManager().mkJsonFile(name, jsonFactory.apply(forest));
						tool.getFileManager().mkDotFile(name, dotFactory.apply(forest));
					} catch (IOException e) {
						throw new RuntimeException(e);
					}

					int i = 1;
					for (DataframeForest sub : subgraphs)
						try {
							String name = "df" + i++ + "@" + node.getLocation();
							if (!result.getId().isStartingId())
								name += "_" + result.getId().hashCode();
							tool.getFileManager().mkJsonFile(name, jsonFactory.apply(sub));
							tool.getFileManager().mkDotFile(name, dotFactory.apply(sub));
						} catch (IOException e) {
							throw new RuntimeException(e);
						}

					for (Entry<Identifier, DataframeForest> sub : vargraphs.entrySet()) {
						Set<Identifier> ids;
						if (reverseMap.containsKey(sub.getKey()))
							ids = reverseMap.get(sub.getKey());
						else
							ids = Collections.singleton(sub.getKey());
						for (Identifier id : ids)
							if (id instanceof Variable)
								try {
									String name = "var_" + id.getName() + "@" + node.getLocation();
									if (!result.getId().isStartingId())
										name += "_" + result.getId().hashCode();
									tool.getFileManager().mkJsonFile(name, jsonFactory.apply(sub.getValue()));
									tool.getFileManager().mkDotFile(name, dotFactory.apply(sub.getValue()));
								} catch (IOException e) {
									throw new RuntimeException(e);
								}
					}
				} catch (SemanticException e) {
					throw new AnalysisExecutionException(e);
				}
		}

		return true;
	}

	private SerializableValue label(
			DataframeOperation op,
			Map<DataframeOperation, Set<Identifier>> refs) {
		String extra = refs.containsKey(op)
				? "\nPointed by: " + refs.get(op).stream().map(id -> id.toString()).sorted()
						.reduce("", (
								res,
								s) -> res + s + "\n")
						.trim()
				: "";
		return new SerializableString(new TreeMap<>(), "at: " + op.getWhere().getCodeLocation() + extra);
	}

	@SuppressWarnings("unchecked")
	private Set<Identifier> reverse(
			PointBasedHeap heap,
			Identifier key) {
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
					SimpleAbstractState<PointBasedHeap, DataframeGraphDomain, TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Edge edge) {
		return true;
	}
}
