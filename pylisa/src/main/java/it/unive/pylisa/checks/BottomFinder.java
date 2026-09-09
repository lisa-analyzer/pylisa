package it.unive.pylisa.checks;

import it.unive.lisa.AnalysisExecutionException;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;

public class BottomFinder
		implements
		SemanticCheck<
				SimpleAbstractState<
						HeapEnvironment<AllocationSites>,
						DataframeGraphDomain,
						TypeEnvironment<TypeSet>>,
				SimpleAbstractDomain<
						HeapEnvironment<AllocationSites>,
						DataframeGraphDomain,
						TypeEnvironment<TypeSet>>> {

	@Override
	public void beforeExecution(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool) {
	}

	@Override
	public void afterExecution(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool) {
	}

	@Override
	public boolean visitUnit(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool,
			Unit unit,
			Global global,
			boolean instance) {
	}

	@Override
	public boolean visit(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool,
			CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool,
			CFG graph,
			Statement node) {
		return true;
	}

	@Override
	public boolean visit(
			SemanticTool<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, DataframeGraphDomain, TypeEnvironment<TypeSet>>> tool,
			CFG graph,
			Edge edge) {
		Statement source = edge.getSource();
		Statement dest = edge.getDestination();

		for (AnalyzedCFG<
				SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain,
						TypeEnvironment<TypeSet>>> res : tool.getResultOf(graph)) {
			AnalysisState<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain,
							TypeEnvironment<TypeSet>>> pre = res.getAnalysisStateAfter(source);
			AnalysisState<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, DataframeGraphDomain,
							TypeEnvironment<TypeSet>>> post = res.getAnalysisStateAfter(dest);

			if (!pre.isBottom() && post.isBottom())
				tool.warnOn(dest, "State goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
			else if (!pre.getExecutionState().heapState.isBottom()
					&& post.getExecutionState().heapState.isBottom())
				tool.warnOn(dest, "Heap goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
			else if (!topOrBottom(pre.getExecutionState().valueState)
					&& topOrBottom(post.getExecutionState().valueState))
				tool.warnOn(dest,
						"DataframeGraphDomain goes to bottom after " + edge.getClass().getSimpleName() + " in "
								+ dest);
		}

		return true;
	}

	private static boolean topOrBottom(
			DataframeGraphDomain dgd) {
		return dgd.graph.isTop() || dgd.graph.isBottom()
				|| dgd.constants.isTop() || dgd.constants.isBottom()
				|| dgd.pointers.isTop() || dgd.pointers.isBottom() || dgd.pointers.getMap().isEmpty()
				|| dgd.operations.isTop() || dgd.operations.isBottom() || dgd.operations.getMap().isEmpty();

	}
}
