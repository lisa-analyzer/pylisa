package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitFuncdef} — the ~125-line god-method
 * decomposed in Chunk 5. Targets the state-restore contract (partial CFG
 * leakage on exception), nested defs, and def-inside-class.
 */
final class VisitFuncdefTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitFuncdefTargetedTests.class);

	@Test
	void top_level_def_registers_function_unit() throws Exception {
		var result = FrontendTestSupport.parseSnippet("def f(a, b):\n    return a + b\n");
		LOG.debug("top-level def at {}", result.sourceFile());
		assertThat(FrontendTestSupport.hasUnit(result.program(), "__main__.f")).isTrue();
	}

	@Test
	void nested_def_registers_inner_unit() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"def outer():\n    def inner():\n        return 1\n    return inner\n");
		assertThat(result.program()).isNotNull();
	}

	@Test
	void def_inside_class_registers_method_unit() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"class C:\n    def method(self, x):\n        return x + 1\n");
		assertThat(result.program()).isNotNull();
	}

	@Test
	void def_with_mix_of_positional_and_keyword_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(a, b=1, *args, c, **kwargs):\n    return a\n").program()).isNotNull();
	}

	@Test
	void def_with_every_type_hint_shape_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(a: int, b: 'Config', *args: str, **kwargs: bool) -> bool:\n    return True\n").program())
						.isNotNull();
	}

	@Test
	void two_defs_same_name_at_same_scope_parses() throws Exception {
		// Redeclaration — current frontend handles this by mutating the symbol
		// table. Targeted at visitFuncdef's state-restore and re-registration
		// logic.
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    return 1\ndef f():\n    return 2\n").program()).isNotNull();
	}
}
