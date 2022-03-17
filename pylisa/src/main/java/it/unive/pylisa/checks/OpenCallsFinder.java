package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractState;
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
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;

public class OpenCallsFinder<A extends AbstractState<A, H, V, T>,
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
		if (node instanceof UnresolvedCall) {
			Call resolved = tool.getResolvedVersion((UnresolvedCall) node);
			if (resolved instanceof OpenCall)
				tool.warnOn(node, "Open call found: " + node);
		}

		return true;
	}

	@Override
	public boolean visit(CheckToolWithAnalysisResults<A, H, V, T> tool, CFG graph, Edge edge) {
		return true;
	}

}
