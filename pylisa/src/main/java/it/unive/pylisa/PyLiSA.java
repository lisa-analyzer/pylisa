package it.unive.pylisa;

import it.unive.lisa.AnalysisException;
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
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.checks.DataframeDumper;
import it.unive.pylisa.checks.DataframeStructureConstructor;
import java.io.IOException;
import org.apache.commons.io.FilenameUtils;

public class PyLiSA {

	public static void main(
			String[] args)
			throws IOException,
			AnalysisException {
		if (args.length < 2) {
			System.err.println("PyLiSA needs two arguments: the file to analyze and the working directory");
			System.exit(-1);
		}

		String file = args[0];
		String workdir = args[1];
		String extension = FilenameUtils.getExtension(file);
		PyFrontend translator = new PyFrontend(file, extension.equals("ipynb"));
		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.optimize = true;
		conf.workdir = workdir;
		conf.jsonOutput = true;
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.semanticChecks.add(new DataframeDumper());
		conf.semanticChecks.add(new DataframeStructureConstructor());

		PointBasedHeap heap = new PyFieldSensitivePointBasedHeap();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		DataframeGraphDomain df = new DataframeGraphDomain();
		conf.abstractState = new SimpleAbstractState<>(heap, df, type);

		LiSA lisa = new LiSA(conf);
		LiSAReport report = lisa.run(program);
		if (!report.getWarnings().isEmpty()) {
			System.out.println("The analysis generated the following warnings:");
			for (Warning w : report.getWarnings())
				System.out.println("  " + w);
		}

	}
}
