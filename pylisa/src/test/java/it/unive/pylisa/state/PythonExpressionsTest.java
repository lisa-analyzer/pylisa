package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks Python's evaluation order and exceptions for expressions: the
 * conditional expression and {@code and} / {@code or} evaluate only what
 * Python evaluates, a chained comparison evaluates each operand once and stops
 * at the first false comparison, unary operators keep Python's types, and
 * arithmetic raises {@code ZeroDivisionError} and {@code TypeError}.
 */
class PythonExpressionsTest {

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void expressionsEvaluateWhatPythonEvaluates(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper.analyse("src/test/resources/programs/python/control_expressions.py", config)
				.assertAllProved();
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void arithmeticRaisesPythonsExceptions(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper
				.analyse("src/test/resources/programs/python/raising_arithmetic.py", config);
		assertRaised(helper, "@zero", "builtins.ZeroDivisionError");
		assertRaised(helper, "@type", "builtins.TypeError");
		assertRaised(helper, "@negation", "builtins.TypeError");
		// dividing by an unknown number may raise, and may not
		assertTrue(helper.after("@division").isReachable());
	}

	private static void assertRaised(
			StateTestHelper helper,
			String label,
			String error) {
		Set<String> errors = helper.after(label).errors();
		assertTrue(errors.contains(error), label + ": " + errors);
	}
}
