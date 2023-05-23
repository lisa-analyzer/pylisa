package it.unive.pylisa;

import it.unive.lisa.program.language.LanguageFeatures;
import it.unive.lisa.program.language.hierarchytraversal.HierarcyTraversalStrategy;
import it.unive.lisa.program.language.hierarchytraversal.SingleInheritanceTraversalStrategy;
import it.unive.lisa.program.language.parameterassignment.ParameterAssigningStrategy;
import it.unive.lisa.program.language.parameterassignment.PythonLikeAssigningStrategy;
import it.unive.lisa.program.language.resolution.ParameterMatchingStrategy;
import it.unive.lisa.program.language.resolution.PythonLikeMatchingStrategy;
import it.unive.lisa.program.language.resolution.RuntimeTypesMatchingStrategy;
import it.unive.lisa.program.language.validation.BaseValidationLogic;
import it.unive.lisa.program.language.validation.ProgramValidationLogic;
import it.unive.pylisa.program.language.resolution.RelaxedRuntimeTypesMatchingStrategy;

public class PythonFeatures extends LanguageFeatures {

	@Override
	public ParameterMatchingStrategy getMatchingStrategy() {
		return new PythonLikeMatchingStrategy(new RelaxedRuntimeTypesMatchingStrategy());
	}

	@Override
	public HierarcyTraversalStrategy getTraversalStrategy() {
		// TODO this is not right, but its fine for now
		return SingleInheritanceTraversalStrategy.INSTANCE;
	}

	@Override
	public ParameterAssigningStrategy getAssigningStrategy() {
		return PythonLikeAssigningStrategy.INSTANCE;
	}

	@Override
	public ProgramValidationLogic getProgramValidationLogic() {
		return new BaseValidationLogic();
	}

}
