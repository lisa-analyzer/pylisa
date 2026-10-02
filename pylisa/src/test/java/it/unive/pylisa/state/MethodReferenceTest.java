package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks what a method reference such as {@code self.cb} evaluates to when it
 * is used as a value rather than called. The value identifies the method of
 * the class; the object it is read from is not kept with it, so calling the
 * value later does not pass the receiver (a known gap of the analysis).
 */
class MethodReferenceTest {

	private static final String PROGRAM = "src/test/resources/programs/python/method_reference.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aMethodPassedAsArgumentIdentifiesTheMethod(
			AnalysisConfig config)
			throws Exception {
		Set<String> types = StateTestHelper.analyse(PROGRAM, config).after("@x").types("x");
		assertEquals(1, types.size(), types.toString());
		assertTrue(types.iterator().next().endsWith(".cb"), types.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aMethodStoredInAVariableIdentifiesTheMethod(
			AnalysisConfig config)
			throws Exception {
		Set<String> types = StateTestHelper.analyse(PROGRAM, config).after("@y").types("y");
		assertEquals(1, types.size(), types.toString());
		assertTrue(types.iterator().next().endsWith(".cb"), types.toString());
	}
}
