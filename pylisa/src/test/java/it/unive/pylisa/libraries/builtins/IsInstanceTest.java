package it.unive.pylisa.libraries.builtins;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that {@code isinstance} is true or false only when every runtime
 * type of its object decides it, and an unknown bool otherwise.
 */
class IsInstanceTest {

	private static final String PROGRAM = "src/test/resources/programs/isinstance/isinstance_cases.py";

	private static final String HIERARCHY = "src/test/resources/programs/isinstance/hierarchy.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void everyRuntimeTypeDecidesTheResult(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.exact(false), helper.after("@str_bytes").value("r1"));
		assertEquals(Val.exact(false), helper.after("@none_bytes").value("r2"));
		assertEquals(Val.exact(true), helper.after("@str_object").value("r3"));
		assertEquals(Val.exact(false), helper.after("@str_or_none_bytes").value("r5"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anObjectOfUnknownTypeGivesAnUnknownResult(
			AnalysisConfig config)
			throws Exception {
		assertEquals(Val.top(), StateTestHelper.analyse(PROGRAM, config).after("@unknown_bytes").value("r4"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aNegatedTestThatAlwaysHoldsKeepsItsBranch(
			AnalysisConfig config)
			throws Exception {
		var taken = StateTestHelper.analyse(PROGRAM, config).after("@taken");
		assertTrue(taken.isReachable());
		assertEquals(Set.of("null", "string"), taken.types("x"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aClassWhoseAncestorsAreNotCertainGivesAnUnknownResult(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(HIERARCHY, config);
		// a base imported from unknown code may derive from bytes
		assertEquals(Val.top(), helper.after("@unresolved_base").value("r1"));
		// the base is a subclass of bytes on one path only
		assertEquals(Val.top(), helper.after("@ambiguous_base").value("r2"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aClassThatMayCheckInstancesItselfGivesAnUnknownResult(
			AnalysisConfig config)
			throws Exception {
		// the metaclass decides, also for a str
		assertEquals(Val.top(), StateTestHelper.analyse(HIERARCHY, config).after("@metaclass").value("r5"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aBaseNameTheFileRebindsGivesAnUnknownResult(
			AnalysisConfig config)
			throws Exception {
		// Base2 is bytes when Derived is defined
		assertEquals(Val.top(), StateTestHelper.analyse(HIERARCHY, config).after("@rebound_base").value("r6"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aClassWithAKnownHierarchyDecidesTheResult(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(HIERARCHY, config);
		assertEquals(Val.exact(true), helper.after("@own_class").value("r3"));
		assertEquals(Val.exact(false), helper.after("@other_class").value("r4"));
	}
}
