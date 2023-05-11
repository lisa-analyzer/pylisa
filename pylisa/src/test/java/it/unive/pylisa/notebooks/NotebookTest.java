package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FilenameUtils;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAReport;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.warnings.Warning;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.outputs.compare.JsonReportComparer;
import it.unive.lisa.outputs.json.JsonReport;
import it.unive.lisa.program.Program;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.checks.BottomFinder;
import it.unive.pylisa.checks.DataframeDumper;
import it.unive.pylisa.checks.DataframeStructureConstructor;
import it.unive.pylisa.checks.OpenCallsFinder;

public abstract class NotebookTest {

	private static final boolean FIND_OPEN_CALLS = false;

	private static String getWorkdir(String file) {
		String kind = FilenameUtils.getExtension(file);
		return "notebook-tests-workdir/" + kind + "/" + FilenameUtils.getBaseName(file);
	}

	protected void perform(String file) throws IOException, AnalysisException {
		perform(file, FIND_OPEN_CALLS);
	}

	protected void perform(String file, Integer... cells) throws IOException, AnalysisException {
		perform(file, FIND_OPEN_CALLS, cells);
	}

	protected void perform(String file, boolean findOpenCalls, Integer... cells) throws IOException, AnalysisException {
		String kind = FilenameUtils.getExtension(file);
		PyFrontend translator = new PyFrontend(file, kind.equals("ipynb"), cells);

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = buildConfig(getWorkdir(file), findOpenCalls);
		LiSA lisa = new LiSA(conf);
		LiSAReport report = lisa.run(program);
		for (Warning warn : report.getWarnings())
			System.err.println(warn);
	}

	private LiSAConfiguration buildConfig(String workdir, boolean findOpenCalls) throws AnalysisSetupException {
		try {
			FileManager.forceDeleteFolder(workdir.toString());
		} catch (IOException e) {
			e.printStackTrace(System.err);
			fail("Cannot delete working directory '" + workdir + "': " + e.getMessage());
		}

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workdir;
		conf.serializeResults = true;
//		conf.analysisGraphs = GraphType.HTML_WITH_SUBNODES;
		conf.jsonOutput = true;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.semanticChecks.add(new DataframeDumper(conf));
		conf.semanticChecks.add(new BottomFinder<>());
		conf.semanticChecks.add(new DataframeStructureConstructor());
		if (findOpenCalls)
			conf.semanticChecks.add(new OpenCallsFinder<>());

		PointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		DataframeGraphDomain df = new DataframeGraphDomain();
		conf.abstractState = new SimpleAbstractState<>(heap, df, type);
		return conf;
	}

	protected void performAndCheck(String file, Integer... cells) throws IOException, AnalysisException {
		performAndCheck(file, false, cells);
	}

	protected void performAndCheck(String file, boolean findOpenCalls, Integer... cells)
			throws IOException, AnalysisException {
		String workdir = getWorkdir(file);
		try {
			FileManager.forceDeleteFolder(workdir);
		} catch (IOException e) {
			e.printStackTrace(System.err);
			fail("Cannot delete working directory '" + workdir + "': " + e.getMessage());
		}

		perform(file, findOpenCalls, cells);

		Path expectedPath = Paths.get(workdir.replace("workdir", "expected"));
		Path actualPath = Paths.get(workdir);

		File expFile = Paths.get(expectedPath.toString(), "report.json").toFile();
		File actFile = Paths.get(actualPath.toString(), "report.json").toFile();
		try (FileReader l = new FileReader(expFile); FileReader r = new FileReader(actFile)) {
			JsonReport expected = JsonReport.read(l);
			JsonReport actual = JsonReport.read(r);
			assertTrue("Results are different",
					JsonReportComparer.compare(expected, actual, expectedPath.toFile(), actualPath.toFile()));
		} catch (FileNotFoundException e) {
			e.printStackTrace(System.err);
			fail("Unable to find report file");
		} catch (IOException e) {
			e.printStackTrace(System.err);
			fail("Unable to compare reports");
		}
	}
}
