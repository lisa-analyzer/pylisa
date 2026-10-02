package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.ErrorSite;
import it.unive.pylisa.testing.Point;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that an error raised while an object is constructed is raised by a
 * statement of the program, so that it can be attributed to the construction
 * and propagated to the callers of the function that constructs the object.
 */
class SyntheticParentTest {

	private static final String IN_HELPER = "src/test/resources/programs/calls/instantiation_in_helper.py";

	private static final String DIRECT = "src/test/resources/programs/calls/instantiation_direct.py";

	private static final String RAISING_NEW = "src/test/resources/programs/calls/instantiation_raising_new.py";

	private static final String MEMBERSHIP = "src/test/resources/programs/calls/membership_in_helper.py";

	private static final String KEYWORD = "src/test/resources/programs/calls/keyword_argument_in_helper.py";

	private static final String VALUE_ERROR = "builtins.ValueError";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void constructorErrorInHelperReachesTheCallerOfTheHelper(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(IN_HELPER, config);
		int helperCall = helper.lineOf("@outer");
		Point afterCall = helper.after("@outer");
		assertTrue(afterCall.errorSites().stream()
				.anyMatch(site -> site.type().equals(VALUE_ERROR) && site.line() == helperCall),
				"the error is re-raised at the call of the helper: " + afterCall.errorSites());
		assertTrue(afterCall.everyErrorRaisedWithinProgram(), afterCall.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void constructorErrorIsRaisedWithinTheProgram(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(DIRECT, config);
		int construction = helper.lineOf("@direct");
		Point after = helper.after("@direct");
		assertTrue(after.errorSites().contains(new ErrorSite(VALUE_ERROR, "testnatives.Widget.__init__", construction)),
				after.errorSites().toString());
		assertTrue(after.everyErrorRaisedWithinProgram(), "raised by a call with no program statement: "
				+ after.errorSites());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void errorOfNewIsRaisedWithinTheProgram(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(RAISING_NEW, config);
		int construction = helper.lineOf("@raw");
		Point after = helper.after("@raw");
		assertTrue(after.errorSites().contains(new ErrorSite(VALUE_ERROR, "testnatives.Raw.__new__", construction)),
				after.errorSites().toString());
		assertTrue(after.everyErrorRaisedWithinProgram(), "raised by a call with no program statement: "
				+ after.errorSites());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void errorInOperandOfMembershipTestReachesTheCallerOfTheHelper(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(MEMBERSHIP, config);
		int helperCall = helper.lineOf("@membership");
		Point afterCall = helper.after("@membership");
		assertTrue(afterCall.errorSites().stream()
				.anyMatch(site -> site.type().equals(VALUE_ERROR) && site.line() == helperCall),
				"the error is re-raised at the call of the helper: " + afterCall.errorSites());
		assertTrue(afterCall.everyErrorRaisedWithinProgram(), afterCall.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void errorInKeywordArgumentReachesTheCallerOfTheHelper(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(KEYWORD, config);
		int helperCall = helper.lineOf("@keyword");
		Point afterCall = helper.after("@keyword");
		assertTrue(afterCall.errorSites().stream()
				.anyMatch(site -> site.type().equals("builtins.ZeroDivisionError") && site.line() == helperCall),
				"the error is re-raised at the call of the helper: " + afterCall.errorSites());
		assertTrue(afterCall.everyErrorRaisedWithinProgram(), afterCall.toString());
	}
}
