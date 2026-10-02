package it.unive.pylisa.libraries.argparse;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.Point;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the argparse model: each argument gives its attribute of the parsed
 * namespace the values it may take, and parsing may exit.
 */
class ArgparseModelTest {

	private static final String PROGRAM = "src/test/resources/programs/argparse/argparse_cases.py";

	private static final String EDGES = "src/test/resources/programs/argparse/argparse_edges.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void argumentsGiveTheirAttributesTheirPossibleTypes(
			AnalysisConfig config)
			throws Exception {
		Point after = StateTestHelper.analyse(PROGRAM, config).after("@after");
		assertEquals(Set.of("bool"), after.types("args.encrypt"), "store_true");
		assertEquals(Set.of("string"), after.types("args.input_file"), "required option");
		assertEquals(Set.of("null", "string"), after.types("args.mac"), "optional, no default");
		assertEquals(Set.of("int32", "string"), after.types("args.n"), "optional with a default");
		assertEquals(Set.of("string"), after.types("args.pos"), "positional");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anActionTheModelDoesNotFollowGivesAnyType(
			AnalysisConfig config)
			throws Exception {
		Set<String> counted = StateTestHelper.analyse(PROGRAM, config).after("@after").types("args.c");
		assertTrue(counted.contains("untyped") || counted.contains("NO_INFO"), counted.toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void argumentsSharingADestKeepTheValuesOfEach(
			AnalysisConfig config)
			throws Exception {
		Point parsed = StateTestHelper.analyse(EDGES, config).after("@same_dest");
		assertEquals(Set.of("bool", "string"), parsed.types("a.speed"));
		// an attribute stored once is still exact
		assertEquals(Set.of("null", "string"), parsed.types("a.out"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aParserThatIsNotFollowedGivesAnUnknownNamespace(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(EDGES, config);
		for (String[] label : new String[][] { { "@after_defaults", "b" }, { "@dest_only", "c" } }) {
			Set<String> types = helper.after(label[0]).types(label[1]);
			assertTrue(types.contains("untyped") || types.contains("NO_INFO"), label[0] + ": " + types);
		}
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void parsingMayExit(
			AnalysisConfig config)
			throws Exception {
		Point parsed = StateTestHelper.analyse(PROGRAM, config).after("@parsed");
		assertTrue(parsed.isReachable());
		assertTrue(parsed.errors().contains("builtins.SystemExit"), parsed.errors().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anExplicitNoneDefaultIsKept(
			AnalysisConfig config)
			throws Exception {
		Point parsed = StateTestHelper.analyse(EDGES, config).after("@store_true_none");
		assertEquals(Set.of("bool", "null"), parsed.types("d.x"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void callsTheModelsDoNotFollowGiveAnUnknownNamespace(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(EDGES, config);
		for (String[] label : new String[][] { { "@subparsers", "e" }, { "@no_exit", "f" }, { "@converter", "g" } }) {
			Set<String> types = helper.after(label[0]).types(label[1]);
			assertTrue(types.contains("untyped") || types.contains("NO_INFO"), label[0] + ": " + types);
		}
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aConverterMayRaiseAnything(
			AnalysisConfig config)
			throws Exception {
		Point parsed = StateTestHelper.analyse(EDGES, config).after("@converter");
		assertTrue(parsed.errors().contains("builtins.BaseException"), parsed.errors().toString());
	}
}
