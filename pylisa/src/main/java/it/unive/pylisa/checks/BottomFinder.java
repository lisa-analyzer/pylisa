package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.CFGWithAnalysisResults;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.analysis.dataframes.graph.DataframeGraphDomain;

public class BottomFinder<A extends AbstractState<A, H, V, T>,
		H extends HeapDomain<H>,
		V extends ValueDomain<V>,
		T extends TypeDomain<T>> implements SemanticCheck<A, H, V, T> {

	@Override
	public void beforeExecution(CheckToolWithAnalysisResults<A, H, V, T> tool) {
	}

	@Override
	public void afterExecution(CheckToolWithAnalysisResults<A, H, V, T> tool) {
	}

	@Override
	public boolean visitCompilationUnit(CheckToolWithAnalysisResults<A, H, V, T> tool, CompilationUnit unit) {
		return true;
	}

	@Override
	public void visitGlobal(CheckToolWithAnalysisResults<A, H, V, T> tool, Unit unit, Global global, boolean instance) {
	}

	@Override
	public boolean visit(CheckToolWithAnalysisResults<A, H, V, T> tool, CFG graph) {
		return true;
	}

	@Override
	public boolean visit(CheckToolWithAnalysisResults<A, H, V, T> tool, CFG graph, Statement node) {
		return true;
	}

	@Override
	public boolean visit(CheckToolWithAnalysisResults<A, H, V, T> tool, CFG graph, Edge edge) {
		Statement source = edge.getSource();
		Statement dest = edge.getDestination();

		for (CFGWithAnalysisResults<A, H, V, T> res : tool.getResultOf(graph)) {
			AnalysisState<A, H, V, T> pre = res.getAnalysisStateAfter(source);
			AnalysisState<A, H, V, T> post = res.getAnalysisStateAfter(dest);

			if (!pre.isBottom() && post.isBottom())
				tool.warnOn(dest, "State goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
			else if (!pre.getDomainInstance(ValueDomain.class).isBottom()
					&& post.getDomainInstance(ValueDomain.class).isBottom())
				tool.warnOn(dest, "Value goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
			else if (!pre.getDomainInstance(HeapDomain.class).isBottom()
					&& post.getDomainInstance(HeapDomain.class).isBottom())
				tool.warnOn(dest, "Heap goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
			else if (!topOrBottom(pre.getDomainInstance(DataframeGraphDomain.class))
					&& topOrBottom(post.getDomainInstance(DataframeGraphDomain.class)))
				tool.warnOn(dest,
						"DataframeGraphDomain goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
		}

		return true;
	}

	private static boolean topOrBottom(DataframeGraphDomain dgd) {
		return dgd.graph.isTop() || dgd.graph.isBottom()
				|| dgd.constants.isTop() || dgd.constants.isBottom()
				|| dgd.pointers.isTop() || dgd.pointers.isBottom() || dgd.pointers.getMap().isEmpty()
				|| dgd.operations.isTop() || dgd.operations.isBottom() || dgd.operations.getMap().isEmpty();

	}
}
