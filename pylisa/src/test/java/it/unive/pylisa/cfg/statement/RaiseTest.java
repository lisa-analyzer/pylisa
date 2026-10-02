package it.unive.pylisa.cfg.statement;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.testing.Point;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that {@code raise} ends every execution that reaches it with an
 * error: of the builtin exception class it names, or of
 * {@code BaseException} when the class is not a known builtin one.
 */
class RaiseTest {

	private static final String PROGRAM = "src/test/resources/programs/raise/raise_cases.py";

	private static final String REBOUND = "src/test/resources/programs/raise/rebound_name.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void raisingABuiltinExceptionRaisesItsType(
			AnalysisConfig config)
			throws Exception {
		Point after = StateTestHelper.analyse(PROGRAM, config).after("@builtin");
		assertFalse(after.isReachable());
		assertTrue(after.errors().contains("builtins.ValueError"), after.errors().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void raisingAnotherClassRaisesBaseException(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		for (String label : new String[] { "@user", "@bare" }) {
			Point after = helper.after(label);
			assertFalse(after.isReachable(), label);
			assertTrue(after.errors().contains("builtins.BaseException"), label + ": " + after.errors());
		}
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theNormalPathAroundAGuardedRaiseContinues(
			AnalysisConfig config)
			throws Exception {
		Point after = StateTestHelper.analyse(PROGRAM, config).after("@guarded");
		assertTrue(after.isReachable());
		assertEquals(Val.exact(1), after.value("r"));
		assertTrue(after.errors().contains("builtins.TypeError"), after.errors().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aBuiltinNameTheFileRebindsRaisesBaseException(
			AnalysisConfig config)
			throws Exception {
		Point after = StateTestHelper.analyse(REBOUND, config).after("@rebound");
		assertTrue(after.errors().contains("builtins.BaseException"), after.errors().toString());
		assertFalse(after.errors().contains("builtins.ValueError"), after.errors().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void everyBuiltinExceptionWithATypeIsRecognised(
			AnalysisConfig config)
			throws Exception {
		Point after = StateTestHelper.analyse(REBOUND, config).after("@not_implemented");
		assertTrue(after.errors().contains("builtins.NotImplementedError"), after.errors().toString());
	}
}
