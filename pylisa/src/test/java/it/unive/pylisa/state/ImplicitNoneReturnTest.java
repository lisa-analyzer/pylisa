package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.type.NullType;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that a function returns {@code None} when its body ends without a
 * {@code return} or with a bare one, as Python does, so that its result is a
 * value and not the absence of one.
 */
class ImplicitNoneReturnTest {

	private static final String PROGRAM = "src/test/resources/programs/python/implicit_none_return.py";

	/** The name of the type of {@code None}: LiSA's null type. */
	private static final String NONE_TYPE = NullType.INSTANCE.toString();

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void functionWithoutReturnReturnsNone(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertTrue(helper.after("@r").types("r").stream().anyMatch(NONE_TYPE::equals),
				helper.after("@r").types("r").toString());
		assertTrue(helper.after("@none").isReachable());
		assertFalse(helper.after("@other").isReachable(), "r is certainly None");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void functionThatMayFallThroughMayReturnNone(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertTrue(helper.after("@maybe").types("m").stream().anyMatch(NONE_TYPE::equals),
				helper.after("@maybe").types("m").toString());
		assertEquals(2, helper.after("@maybe").types("m").size(), "an int or None");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void bareReturnReturnsNone(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertTrue(helper.after("@bare").types("b").stream().anyMatch(NONE_TYPE::equals),
				helper.after("@bare").types("b").toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void methodAndFunctionWithSeveralExitsReturnNone(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/python/implicit_none_methods.py",
				config);
		assertEquals(Set.of(NONE_TYPE), helper.after("@method").types("p"));
		assertEquals(Set.of(NONE_TYPE), helper.after("@branches").types("q"));
	}

	@Test
	void generatorIsMarkedAsTranslatedUnsoundly() throws Exception {
		Program translated = new PyFrontend("src/test/resources/programs/python/generator.py", false)
				.toLiSAProgram(true);
		boolean unsound = false;
		for (CodeMember member : translated.getCodeMembersRecursively())
			for (Annotation annotation : member.getDescriptor().getAnnotations())
				unsound |= annotation.getAnnotationName().equals(ParserSupport.UNSOUND_TRANSLATION);
		assertTrue(unsound, "a generator's call does not return what its body computes");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void generatorCallDoesNotReturnNone(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/python/generator.py", config);
		assertNotEquals(Set.of(NONE_TYPE), helper.after("@generator").types("g"),
				"a generator's call returns a generator object");
		assertNotEquals(Set.of(NONE_TYPE), helper.after("@generator_return").types("h"));
	}
}
