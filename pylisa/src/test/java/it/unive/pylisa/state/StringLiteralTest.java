package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that string literals with a prefix never evaluate to their own source
 * text: formatted strings and bytes are unknown values, while raw and unicode
 * strings evaluate to their contents.
 */
class StringLiteralTest {

	private static final String PROGRAM = "src/test/resources/programs/python/string_literals.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void formattedStringsAreUnknown(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.top(), helper.after("@formatted").value("t"));
		assertEquals(Val.top(), helper.after("@formatted_upper").value("u"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void rawAndUnicodeStringsKeepTheirContents(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.exact("/a\\b"), helper.after("@raw").value("raw"));
		assertEquals(Val.exact("/plain"), helper.after("@unicode_prefix").value("plain"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void bytesAreUnknown(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.top(), helper.after("@bytes").value("data"));
	}
}
