package it.unive.pylisa.outputs;

import it.unive.lisa.LiSAReport;
import it.unive.lisa.ReportingTool;
import it.unive.lisa.outputs.LiSAOutput;
import it.unive.lisa.program.Application;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.util.file.FileManager;
import java.io.IOException;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class ApplicationStructure
		implements
		LiSAOutput {

	private static final Logger LOG = LogManager.getLogger(ApplicationStructure.class);

	/**
	 * The name of the json report that LiSA can optionally dump.
	 */
	public static final String REPORT_NAME = "application-report.txt";

	@Override
	public void dump(
			Application app,
			LiSAReport report,
			ReportingTool tool,
			FileManager fileManager)
			throws IOException {
		LOG.info("Dumping analysis report to '" + REPORT_NAME + "'");
		fileManager.mkOutputFile(REPORT_NAME, writer -> {
			for (Program p : app.getPrograms()) {
				writer.write("PROGRAM " + p.getName());
				writer.write(System.lineSeparator());

				writer.write("\tUNITS");
				writer.write(System.lineSeparator());

				for (Unit u : p.getUnits()) {
					writer.write("\t\t" + u.getName() + ": " + u.getClass().getSimpleName());
					writer.write(System.lineSeparator());
					if (!u.getCodeMembers().isEmpty()) {
						writer.write("\t\t\tCODE MEMBERS");
						writer.write(System.lineSeparator());
						for (CodeMember cm : u.getCodeMembers()) {
							writer.write("\t\t\t\t" + cm.getDescriptor());
							writer.write(System.lineSeparator());
						}
					}
					if (!u.getGlobals().isEmpty()) {
						writer.write("\t\t\tGLOBALS");
						writer.write(System.lineSeparator());
						for (Global g : u.getGlobals()) {
							writer.write("\t\t\t\t" + g.getName());
							writer.write(System.lineSeparator());
						}
					}
				}
			}
			LOG.info("Report file dumped to '" + REPORT_NAME + "'");
		});
	}

	@Override
	public boolean isReportOutput() {
		return true;
	}
}
