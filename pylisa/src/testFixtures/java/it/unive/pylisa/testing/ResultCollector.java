package it.unive.pylisa.testing;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.OptimizedAnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.analysis.ValueReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * A semantic check that does not check anything: it keeps the per-statement
 * results of every analysed CFG, which LiSA otherwise does not return to its
 * caller, so that tests can inspect the state at any program point once the
 * analysis is over.
 *
 * @param <A> the kind of abstract state
 * @param <D> the kind of abstract domain
 */
final class ResultCollector<A extends AbstractLattice<A>, D extends AbstractDomain<A>> implements SemanticCheck<A, D> {

	private final Map<CFG, Collection<AnalyzedCFG<A>>> results = new HashMap<>();

	private Analysis<A, D> analysis;

	@Override
	public void beforeExecution(
			SemanticTool<A, D> tool) {
		analysis = tool.getAnalysis();
	}

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph) {
		Collection<AnalyzedCFG<A>> analyzed = tool.getResultOf(graph);
		for (AnalyzedCFG<A> result : analyzed)
			if (result instanceof OptimizedAnalyzedCFG)
				throw new IllegalStateException("The results of " + graph
						+ " are optimized: states of inner statements are not stored and cannot be inspected");
		results.put(graph, List.copyOf(analyzed));
		return true;
	}

	/**
	 * Yields the statements of every analysed CFG, including those of CFGs no
	 * execution reached.
	 *
	 * @return the statements
	 */
	Collection<Statement> statements() {
		List<Statement> statements = new ArrayList<>();
		results.keySet().forEach(cfg -> statements.addAll(cfg.getNodes()));
		return statements;
	}

	/**
	 * Yields the state after a statement, joined over every context in which
	 * its CFG was analysed.
	 *
	 * @param statement the statement
	 * @param reader    the reader for the configured value domain
	 *
	 * @return the view of the joined state, or empty if no context of the
	 *             CFG of the statement was analysed
	 */
	Optional<StateView<A, D>> after(
			Statement statement,
			ValueReader reader) {
		AnalysisState<A> joined = null;
		for (AnalyzedCFG<A> result : results.getOrDefault(statement.getCFG(), List.of()))
			try {
				AnalysisState<A> after = result.getAnalysisStateAfter(statement);
				joined = joined == null ? after : joined.lub(after);
			} catch (SemanticException e) {
				throw new IllegalStateException("Cannot join the states after " + statement, e);
			}
		if (joined == null)
			return Optional.empty();
		return Optional.of(new StateView<>(analysis, joined, statement, reader));
	}
}
