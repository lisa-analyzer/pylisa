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
import java.io.Writer;

/**
 * A post-analysis output that produces a single {@code final-network.html} file
 * containing only a Mermaid {@code graph TD} diagram of the <em>lubbed</em>
 * network state across all analyzed CFGs (main analysis + second-pass handler
 * analysis). The file has no CFG panel — just the network topology with HTTP
 * endpoints, S3/ROS2 resources, and cross-protocol boundary edges.
 *
 * @param <A> the kind of {@link AbstractLattice} produced by the domain
 *                {@code D}
 * @param <D> the kind of {@link AbstractDomain} to run during the analysis
 */
public class FinalNetworkMermaidResults<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
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

		// Lub the execution state from every analyzed CFG's exit point.
		// This combines:
		// - HTTP endpoints + handlerMap (from the $init main pass)
		// - S3/ROS2 endpoints + crossEdges (from the handler second-pass)
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
		String mermaid = NetworkMermaidSerializer.toMermaidFromState(stateValue);
		String endpointList = NetworkMermaidSerializer.toEndpointListHtmlFromState(stateValue);

		fileManager.mkHtmlFile("final-network", writer -> buildHtml(mermaid, endpointList, writer));
		// Sibling plain-text Mermaid source so that very large graphs (which
		// the in-browser renderer may refuse to draw) can be rendered offline
		// at arbitrary resolution via the Mermaid CLI, e.g.
		// mmdc -i final-network.mmd -o final-network.pdf
		fileManager.mkOutputFile("final-network.mmd", writer -> writer.write(mermaid));
		fileManager.usedHtmlViewer();
	}

	private static void buildHtml(
			String mermaid,
			String endpointList,
			Writer writer)
			throws IOException {
		writer.write("<!DOCTYPE html>\n"
				+ "<html>\n"
				+ "<head>\n"
				+ "  <meta charset=\"UTF-8\">\n"
				+ "  <title>Final Network</title>\n"
				+ "  <script src=\"https://cdn.jsdelivr.net/npm/mermaid/dist/mermaid.min.js\"></script>\n"
				+ "  <style>\n"
				+ "    body { font-family: sans-serif; margin: 20px; background: #f8f9fa; }\n"
				+ "    h2 { color: #2d3436; }\n"
				+ "    .mermaid { background: #fff; border-radius: 8px; "
				+ "padding: 20px; box-shadow: 0 2px 8px rgba(0,0,0,.1); }\n"
				+ "    .mermaid svg { max-width: 100%; height: auto; }\n"
				+ "    .toolbar { display: flex; gap: 8px; flex-wrap: wrap; margin: 0 0 12px 0; }\n"
				+ "    .toolbar button { font: inherit; padding: 6px 12px; border: 1px solid #b2bec3;"
				+ " border-radius: 4px; background: #fff; cursor: pointer; }\n"
				+ "    .toolbar button:hover { background: #ecf0f1; }\n"
				+ "    .toolbar-hint { color:#636e72; font-size:.85em; margin-top:4px; }\n"
				+ "    @media print {\n"
				+ "      body { background: #fff; margin: 0; }\n"
				+ "      .toolbar, .toolbar-hint { display: none; }\n"
				+ "      .mermaid { box-shadow: none; padding: 0; border-radius: 0; }\n"
				+ "      .mermaid svg { max-width: none; width: 100%; }\n"
				+ "    }\n"
				+ "  </style>\n"
				+ "</head>\n"
				+ "<body>\n"
				+ "  <h2>Network</h2>\n"
				+ "  <div class=\"toolbar\">\n"
				+ "    <button onclick=\"window.print()\">Print / Save as PDF</button>\n"
				+ "    <button onclick=\"downloadSvg()\">Download SVG</button>\n"
				+ "    <button onclick=\"downloadMmd()\">Download Mermaid source</button>\n"
				+ "  </div>\n"
				+ "  <div class=\"toolbar-hint\">Graph too big to render? Use the .mmd file:"
				+ " <code>mmdc -i final-network.mmd -o final-network.pdf</code></div>\n"
				+ "  <div class=\"mermaid\" id=\"graph\">\n"
				+ mermaid + "\n"
				+ "  </div>\n"
				+ (endpointList.isEmpty() ? "" : endpointList)
				+ "  <script id=\"mermaid-source\" type=\"text/plain\">"
				+ escapeForScriptTag(mermaid) + "</script>\n"
				+ "  <script>\n"
				+ "    mermaid.initialize({ startOnLoad: true, theme: 'default' });\n"
				+ "    function downloadBlob(text, filename, mime) {\n"
				+ "      const blob = new Blob([text], { type: mime });\n"
				+ "      const url = URL.createObjectURL(blob);\n"
				+ "      const a = document.createElement('a');\n"
				+ "      a.href = url; a.download = filename;\n"
				+ "      document.body.appendChild(a); a.click();\n"
				+ "      document.body.removeChild(a); URL.revokeObjectURL(url);\n"
				+ "    }\n"
				+ "    function downloadSvg() {\n"
				+ "      const svg = document.querySelector('#graph svg');\n"
				+ "      if (!svg) { alert('Graph has not rendered — download Mermaid source and use mmdc instead.'); return; }\n"
				+ "      const serializer = new XMLSerializer();\n"
				+ "      let src = serializer.serializeToString(svg);\n"
				+ "      if (!src.match(/^<svg[^>]+xmlns=/)) src = src.replace(/^<svg/, '<svg xmlns=\"http://www.w3.org/2000/svg\"');\n"
				+ "      downloadBlob(src, 'final-network.svg', 'image/svg+xml;charset=utf-8');\n"
				+ "    }\n"
				+ "    function downloadMmd() {\n"
				+ "      const txt = document.getElementById('mermaid-source').textContent;\n"
				+ "      downloadBlob(txt, 'final-network.mmd', 'text/plain;charset=utf-8');\n"
				+ "    }\n"
				+ "  </script>\n"
				+ "</body>\n"
				+ "</html>\n");
	}

	/**
	 * Escapes the sequence {@code </script>} so a raw Mermaid source can be
	 * safely embedded inside a {@code <script type="text/plain">} element.
	 */
	private static String escapeForScriptTag(
			String s) {
		return s.replace("</script", "<\\/script");
	}
}
