package it.unive.pylisa;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.network.NetworkAwareAbstractDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.types.PythonInferredTypes;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.interprocedural.NetworkAwareContextBasedAnalysis;
import it.unive.pylisa.outputs.ApplicationStructure;
import it.unive.pylisa.outputs.FinalNetworkMermaidResults;

public class PyLiSA {

	public static void main(
			String[] args)
			throws Exception {
		String mainFile = null;
		String projectDir = null;
		String outputDir = null;

		for (int i = 0; i < args.length - 1; i++) {
			switch (args[i]) {
			case "--main-file":
				mainFile = args[++i];
				break;
			case "--project-dir":
				projectDir = args[++i];
				break;
			case "--output-dir":
				outputDir = args[++i];
				break;
			default:
				break;
			}
		}

		if (mainFile == null || projectDir == null || outputDir == null) {
			System.err.println("Usage: pylisa --main-file <path> --project-dir <path> --output-dir <path>");
			System.exit(1);
		}

		PyFrontend frontend = new PyFrontend(mainFile, false, projectDir);
		Program program = frontend.toLiSAProgram(true);

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = outputDir;
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new ApplicationStructure());
		conf.outputs.add(new FinalNetworkMermaidResults<>());
		conf.interproceduralAnalysis = new NetworkAwareContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		conf.analysis = new NetworkAwareAbstractDomain<>(
				new SimpleAbstractDomain<>(heap, new ConstantPropagation(), new PythonInferredTypes()),
				NetworkAwareContextBasedAnalysis.pyFunctionNameExtractor());

		new LiSA(conf).run(program);
	}
}
