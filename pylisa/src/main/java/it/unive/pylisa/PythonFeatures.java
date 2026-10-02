package it.unive.pylisa;

import it.unive.lisa.program.language.LanguageFeatures;
import it.unive.lisa.program.language.hierarchytraversal.HierarchyTraversalStrategy;
import it.unive.lisa.program.language.hierarchytraversal.SingleInheritanceTraversalStrategy;
import it.unive.lisa.program.language.parameterassignment.ParameterAssigningStrategy;
import it.unive.lisa.program.language.resolution.ParameterMatchingStrategy;
import it.unive.lisa.program.language.validation.ProgramValidationLogic;
import it.unive.pylisa.program.language.parameterassignment.PyAssigningStrategy;
import it.unive.pylisa.program.language.parameterassignment.PyMatchingStrategy;
import it.unive.pylisa.program.language.resolution.RelaxedRuntimeTypesMatchingStrategy;

/**
 * Python language features configuration for LiSA. Wires together the parameter
 * matching, assigning, hierarchy traversal, and program validation strategies
 * that reflect Python semantics.
 */
public class PythonFeatures extends LanguageFeatures {

	@Override
	public ParameterMatchingStrategy getMatchingStrategy() {
		return new PyMatchingStrategy(new RelaxedRuntimeTypesMatchingStrategy());
	}

	@Override
	public HierarchyTraversalStrategy getTraversalStrategy() {
		// TODO this is not right, but its fine for now
		return SingleInheritanceTraversalStrategy.INSTANCE;
	}

	@Override
	public ParameterAssigningStrategy getAssigningStrategy() {
		return PyAssigningStrategy.INSTANCE;
	}

	@Override
	public ProgramValidationLogic getProgramValidationLogic() {
		return new PythonValidationLogic();
	}

}
