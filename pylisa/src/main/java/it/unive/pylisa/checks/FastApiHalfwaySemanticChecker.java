package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;

public class FastApiHalfwaySemanticChecker<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		SemanticCheck<A, D> {

	// private List<Endpoint> endpoints;

	@Override
	public void beforeExecution(
			SemanticTool<A, D> tool) {
		// endpoints = new ArrayList<>();
	}

	@Override
	public void afterExecution(
			SemanticTool<A, D> tooll) {
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
