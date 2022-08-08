package it.unive.pylisa.notebooks;

import static it.unive.lisa.LiSAFactory.getDefaultFor;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.TreeSet;

import org.apache.commons.io.FilenameUtils;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.LiSAConfiguration.GraphType;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.warnings.Warning;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.outputs.compare.JsonReportComparer;
import it.unive.lisa.outputs.json.JsonReport;
import it.unive.lisa.program.Program;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.PyRTA;
import it.unive.pylisa.analysis.dataframes.graph.DataframeGraphDomain;
import it.unive.pylisa.checks.BottomFinder;
import it.unive.pylisa.checks.DataframeDumper;
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
		lisa.run(program);
		Collection<Warning> warnings = new TreeSet<>(lisa.getWarnings());
		for (Warning warn : warnings)
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
		conf.setWorkdir(workdir);
		conf.setSerializeResults(true);
//		conf.setDumpAnalysis(GraphType.HTML_WITH_SUBNODES);
		conf.setJsonOutput(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());
		conf.setCallGraph(new PyRTA());
		conf.setOpenCallPolicy(ReturnTopPolicy.INSTANCE);
		conf.addSemanticCheck(new DataframeDumper(conf));
		conf.addSemanticCheck(new BottomFinder<>());
		if (findOpenCalls)
			conf.addSemanticCheck(new OpenCallsFinder<>());

		PointBasedHeap heap = new FieldSensitivePointBasedHeap();
		InferredTypes type = new InferredTypes();
		DataframeGraphDomain df = new DataframeGraphDomain();
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, df, type));
		return conf;
	}

	protected void performAndCheck(String file, Integer... cells) throws IOException, AnalysisException {
		performAndCheck(file, false, cells);
	}

	protected void performAndCheck(String file, boolean findOpenCalls, Integer... cells) throws IOException, AnalysisException {
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
