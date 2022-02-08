package it.unive.pylisa;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

import java.io.IOException;

import org.apache.commons.io.FilenameUtils;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.program.Program;

public abstract class NotebookTest {

	protected <V extends ValueDomain<V>> void perform(String file, String kind, V value)
			throws IOException, AnalysisException {
		PyToCFG translator = new PyToCFG(file);

		Program program = translator.toLiSAProgram();
		PyToCFG.setupProgram(program);

		LiSAConfiguration conf = buildConfig("notebooks/" + kind + "/" + FilenameUtils.getBaseName(file), value);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	private <V extends ValueDomain<V>> LiSAConfiguration buildConfig(String subPath, V value)
			throws AnalysisSetupException {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.setWorkdir("workdir/" + subPath);
		conf.setDumpTypeInference(true);
		conf.setDumpAnalysis(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());
		conf.setOpenCallPolicy(ReturnTopPolicy.INSTANCE);

		PointBasedHeap heap = new FieldSensitivePointBasedHeap();
		InferredTypes type = new InferredTypes();
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, value, type));
		return conf;
	}
}
