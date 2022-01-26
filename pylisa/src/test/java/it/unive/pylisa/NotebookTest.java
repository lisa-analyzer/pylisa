package it.unive.pylisa;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.combination.ValueCartesianProduct;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.DataframeTransformationDomain;
import it.unive.pylisa.analysis.LibraryDomain;
import it.unive.pylisa.analysis.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.cfg.type.PyLibraryType;
import it.unive.pylisa.cfg.type.PyListType;
import java.io.IOException;
import org.apache.commons.io.FilenameUtils;

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
		conf.setInferTypes(true);
		conf.setDumpAnalysis(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());

		ValueCartesianProduct<ValueEnvironment<LibraryDomain>,
				ValueEnvironment<DataframeTransformationDomain>> domain = new ValueCartesianProduct<>(
						new ValueEnvironment<>(new LibraryDomain("").top()),
						new ValueEnvironment<>(new DataframeTransformationDomain(null)));
		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, domain));
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
