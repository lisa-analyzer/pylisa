package it.unive.pylisa;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

import java.io.IOException;

import org.apache.commons.io.FilenameUtils;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.DataframeTransformationDomain;
import it.unive.pylisa.cfg.type.PyLibraryType;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public abstract class NotebookTest {

	protected void perform(String file, String kind) throws IOException, AnalysisException {
		PyToCFG translator = new PyToCFG(file);

		Program program = translator.toLiSAProgram();
		setupProgram(program);

		LiSAConfiguration conf = buildConfig("notebooks/" + kind + "/" + FilenameUtils.getBaseName(file));
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	private LiSAConfiguration buildConfig(String subPath) throws AnalysisSetupException {
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.setDumpCFGs(true);
		conf.setWorkdir("workdir/" + subPath);
		conf.setDumpTypeInference(true);
		conf.setDumpAnalysis(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());
		conf.setOpenCallPolicy(ReturnTopPolicy.INSTANCE);

		ValueEnvironment<DataframeTransformationDomain> domain = new ValueEnvironment<>(
				new DataframeTransformationDomain(null));
		PointBasedHeap heap = new PointBasedHeap();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, domain, type));
		return conf;
	}

	private void setupProgram(Program program) {
		program.registerType(PyListType.INSTANCE);
		program.registerType(BoolType.INSTANCE);
		program.registerType(StringType.INSTANCE);

		for (CompilationUnit lib : LibrarySpecificationProvider.getLibraryUnits()) {
			PyLibraryType.addUnit(lib);
			program.registerType(new PyLibraryType(lib.getName()));
		}

		LibrarySpecificationProvider.getAllStandardLibraryMethods(program).forEach(program::addConstruct);

		for (CFG cfg : program.getCFGs())
			if (cfg.getDescriptor().getName().equals("main"))
				program.addEntryPoint(cfg);
	}
}
