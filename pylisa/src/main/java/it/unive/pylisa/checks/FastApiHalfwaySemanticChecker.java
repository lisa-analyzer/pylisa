package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;

public class FastApiHalfwaySemanticChecker<A extends AbstractState<A>> implements SemanticCheck<A> {

	// private List<Endpoint> endpoints;

	@Override
	public void beforeExecution(
			CheckToolWithAnalysisResults<A> tool) {
		// endpoints = new ArrayList<>();
	}

	@Override
	public void afterExecution(
			CheckToolWithAnalysisResults<A> tooll) {
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
	public boolean visit(
			CheckToolWithAnalysisResults<A> tool,
			CFG graph,
			Edge edge) {

		return true;
	}
}