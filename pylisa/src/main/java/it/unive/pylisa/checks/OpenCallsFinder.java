package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;

public class OpenCallsFinder<A extends AbstractLattice<A>, D extends AbstractDomain<A>> implements SemanticCheck<A, D> {

	@Override
	public void beforeExecution(
			SemanticTool<A, D> tool) {
	}

	@Override
	public void afterExecution(
			SemanticTool<A, D> tool) {
	}

	@Override
	public boolean visitUnit(
			SemanticTool<A, D> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			SemanticTool<A, D> tool,
			Unit unit,
			Global global,
			boolean instance) {
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
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
			SemanticTool<A, D> tool,
			CFG graph,
			Edge edge) {
		return true;
	}

}
