package it.unive.pylisa.notebooks;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.NonRelationalValueDomain;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.dataframes.DataframeAwareDomain;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import java.io.IOException;
import org.apache.commons.io.FilenameUtils;

public abstract class NotebookTest {

	protected <T extends NonRelationalValueDomain<T> & DataframeAwareDomain<T, D>,
			D extends NonRelationalValueDomain<D>> void perform(String file, String kind, T value)
					throws IOException, AnalysisException {
		PyFrontend translator = new PyFrontend(file, kind.equals("ipynb"));

		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = buildConfig("notebooks/" + kind + "/" + FilenameUtils.getBaseName(file), value);
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	private <T extends NonRelationalValueDomain<T> & DataframeAwareDomain<T, D>,
			D extends NonRelationalValueDomain<D>> LiSAConfiguration buildConfig(String subPath, T value)
					throws AnalysisSetupException {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.setWorkdir("workdir/" + subPath);
		conf.setDumpTypeInference(true);
		conf.setDumpAnalysis(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());
		conf.setOpenCallPolicy(ReturnTopPolicy.INSTANCE);

		PointBasedHeap heap = new FieldSensitivePointBasedHeap();
		InferredTypes type = new InferredTypes();
		SideEffectAwareDataframeDomain<T, D> sea = new SideEffectAwareDataframeDomain<>(value);
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, sea, type));
		return conf;
	}
}
