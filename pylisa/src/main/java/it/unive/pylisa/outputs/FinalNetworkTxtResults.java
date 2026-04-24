package it.unive.pylisa.outputs;

import it.unive.lisa.LiSAReport;
import it.unive.lisa.ReportingTool;
import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.outputs.LiSAOutput;
import it.unive.lisa.outputs.serializableGraph.SerializableValue;
import it.unive.lisa.program.Application;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.util.file.FileManager;
import java.io.IOException;

/**
 * A post-analysis output that produces a single {@code final-network.txt} file
 * containing a plain-text summary of the <em>lubbed</em> network state across
 * all analyzed CFGs. The file is a compact dump of HTTP endpoints grouped by
 * method, with each path annotated by its allocation-site location.
 * <p>
 * Register this instead of (or alongside) {@link FinalNetworkMermaidResults}
 * when a text-only artifact is preferred — for example, when diffing network
 * shapes across runs, or when the mermaid/html output is not needed.
 *
 * @param <A> the kind of {@link AbstractLattice} produced by the domain
 *                {@code D}
 * @param <D> the kind of {@link AbstractDomain} to run during the analysis
 */
public class FinalNetworkTxtResults<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		LiSAOutput {

	@Override
	@SuppressWarnings("unchecked")
	public void dump(
			Application app,
			LiSAReport report,
			ReportingTool tool,
			FileManager fileManager)
			throws IOException {
		if (!(tool instanceof SemanticTool))
			return;

		SemanticTool<A, D> ctool = (SemanticTool<A, D>) tool;

		A combined = null;
		for (CFG cfg : app.getAllCFGs()) {
			for (AnalyzedCFG<A> result : ctool.getResultOf(cfg)) {
				try {
					A execState = result.getExitState().getExecutionState();
					combined = (combined == null) ? execState : combined.lub(execState);
				} catch (SemanticException e) {
					// skip — conservative: keep whatever we have
				}
			}
		}

		if (combined == null)
			return;

		SerializableValue stateValue = combined.representation().toSerializableValue();
		String txt = NetworkMermaidSerializer.toNetworkTxtFromState(stateValue);

		fileManager.mkOutputFile("final-network.txt", writer -> writer.write(txt));
	}
}
