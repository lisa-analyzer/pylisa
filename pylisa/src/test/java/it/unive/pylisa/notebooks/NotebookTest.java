package it.unive.pylisa.notebooks;

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
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;

public abstract class NotebookTest {

	protected void perform(String file) throws IOException, AnalysisException {
		String kind = FilenameUtils.getExtension(file);
		PyFrontend translator = new PyFrontend(file, kind.equals("ipynb"));

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = buildConfig("notebooks/" + kind + "/" + FilenameUtils.getBaseName(file));
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	private LiSAConfiguration buildConfig(String subPath) throws AnalysisSetupException {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.setWorkdir("workdir/" + subPath);
		conf.setDumpTypeInference(true);
		conf.setDumpAnalysis(true);
		conf.setJsonOutput(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());
		conf.setOpenCallPolicy(ReturnTopPolicy.INSTANCE);

		PointBasedHeap heap = new FieldSensitivePointBasedHeap();
		InferredTypes type = new InferredTypes();
		SideEffectAwareDataframeDomain df = new SideEffectAwareDataframeDomain();
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, df, type));
		return conf;
	}
}
