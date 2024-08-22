package it.unive.pylisa.checks;

import it.unive.lisa.AnalysisExecutionException;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;

public class BottomFinder<A extends AbstractState<A>> implements SemanticCheck<A> {

	@Override
	public void beforeExecution(
			CheckToolWithAnalysisResults<A> tool) {
	}

	@Override
	public void afterExecution(
			CheckToolWithAnalysisResults<A> tool) {
	}

	@Override
	public boolean visitUnit(
			CheckToolWithAnalysisResults<A> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			CheckToolWithAnalysisResults<A> tool,
			Unit unit,
			Global global,
			boolean instance) {
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<A> tool,
			CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<A> tool,
			CFG graph,
			Statement node) {
		return true;
	}

	@Override
	@SuppressWarnings("unchecked")
	public boolean visit(
			CheckToolWithAnalysisResults<A> tool,
			CFG graph,
			Edge edge) {
		Statement source = edge.getSource();
		Statement dest = edge.getDestination();

		for (AnalyzedCFG<A> res : tool.getResultOf(graph))
			try {
				AnalysisState<A> pre = res.getAnalysisStateAfter(source);
				AnalysisState<A> post = res.getAnalysisStateAfter(dest);

				if (!pre.isBottom() && post.isBottom())
					tool.warnOn(dest, "State goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
				else if (!pre.getState().getDomainInstance(HeapDomain.class).isBottom()
						&& post.getState().getDomainInstance(HeapDomain.class).isBottom())
					tool.warnOn(dest, "Heap goes to bottom after " + edge.getClass().getSimpleName() + " in " + dest);
				else if (!topOrBottom(pre.getState().getDomainInstance(DataframeGraphDomain.class))
						&& topOrBottom(post.getState().getDomainInstance(DataframeGraphDomain.class)))
					tool.warnOn(dest,
							"DataframeGraphDomain goes to bottom after " + edge.getClass().getSimpleName() + " in "
									+ dest);
			} catch (SemanticException e) {
				throw new AnalysisExecutionException(e);
			}

		return true;
	}

	private static boolean topOrBottom(
			DataframeGraphDomain dgd) {
		return dgd.graph.isTop() || dgd.graph.isBottom()
				|| dgd.constants.isTop() || dgd.constants.isBottom()
				|| dgd.v.isTop() || dgd.v.isBottom() || dgd.v.getMap().isEmpty()
				|| dgd.l.isTop() || dgd.l.isBottom() || dgd.l.getMap().isEmpty();

	}
}
