package it.unive.pylisa.state;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that the arguments of a call are bound to the parameters of the
 * callee as Python binds them, in particular when the call relies on default
 * values and passes no keyword argument.
 */
class BindingTest {

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void argumentsAreBoundAsInPython(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper.analyse("src/test/resources/programs/python/argument_binding.py", config).assertAllProved();
	}
}
