package it.unive.pylisa.state;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks assignments: in a chained assignment the value, the last part, is
 * evaluated and assigned to the first target.
 */
class AssignmentTest {

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theValueOfAChainedAssignmentIsEvaluated(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper.analyse("src/test/resources/programs/python/chained_assignment.py", config)
				.assertAllProved();
	}
}
