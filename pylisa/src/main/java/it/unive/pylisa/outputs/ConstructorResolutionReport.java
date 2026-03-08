package it.unive.pylisa.outputs;

import it.unive.lisa.LiSAReport;
import it.unive.lisa.ReportingTool;
import it.unive.lisa.outputs.LiSAOutput;
import it.unive.lisa.program.Application;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.debug.ConstructorResolutionTrace;
import it.unive.pylisa.debug.ConstructorResolutionTrace.ResolutionEvent;
import java.io.IOException;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class ConstructorResolutionReport implements LiSAOutput {

	private static final Logger LOG = LogManager.getLogger(ConstructorResolutionReport.class);
	public static final String REPORT_NAME = "constructor-resolution-report.txt";

	@Override
	public void dump(
			Application app,
			LiSAReport report,
			ReportingTool tool,
			FileManager fileManager)
			throws IOException {
		List<ResolutionEvent> events = ConstructorResolutionTrace.snapshotAndClear();
		LOG.info("Dumping constructor resolution report to '{}'", REPORT_NAME);
		fileManager.mkOutputFile(REPORT_NAME, writer -> {
			writer.write("EVENTS " + events.size());
			writer.write(System.lineSeparator());
			for (ResolutionEvent event : events) {
				writer.write("site=" + event.site());
				writer.write(System.lineSeparator());
				writer.write("  class=" + event.className());
				writer.write(System.lineSeparator());
				writer.write("  attribute=" + event.attribute());
				writer.write(System.lineSeparator());
				writer.write("  immediate-ancestors=" + String.join(",", event.immediateAncestors()));
				writer.write(System.lineSeparator());
				writer.write("  lookup-path=" + event.lookupPath());
				writer.write(System.lineSeparator());
				writer.write("  runtime-types=" + event.runtimeTypes());
				writer.write(System.lineSeparator());
				writer.write("  resolved-owner=" + event.resolvedOwner());
				writer.write(System.lineSeparator());
				writer.write("  mode=" + event.mode());
				writer.write(System.lineSeparator());
			}
		});
	}

	@Override
	public boolean isReportOutput() {
		return true;
	}
}
