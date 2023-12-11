package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;

public class OpenCallsFinder<A extends AbstractState<A>> implements SemanticCheck<A> {

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
		if (node instanceof UnresolvedCall)
			for (AnalyzedCFG<A> result : tool.getResultOf(graph))
				try {
					Call resolved = tool.getResolvedVersion((UnresolvedCall) node, result);
					if (resolved instanceof OpenCall)
						tool.warnOn(node, "Open call found: " + node);
				} catch (SemanticException e) {
					throw new RuntimeException(e);
				}

		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<A> tool,
			CFG graph,
			Edge edge) {
		return true;
	}

}
