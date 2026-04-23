package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: decorator forms — plain, dotted, with-args, stacked, class-vs-fn.
 */
final class DecoratorTests {

	private static final Logger LOG = LogManager.getLogger(DecoratorTests.class);

	@Test
	void simple_decorator_on_function_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("@foo\ndef f():\n    pass\n");
		LOG.debug("simple decorator parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void dotted_decorator_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("@foo.bar\ndef f():\n    pass\n").program()).isNotNull();
	}

	@Test
	void decorator_with_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("@foo('a', x=1)\ndef f():\n    pass\n").program()).isNotNull();
	}

	@Test
	void multiple_stacked_decorators_parse() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"@first\n@second\n@third\ndef f():\n    pass\n").program()).isNotNull();
	}

	@Test
	void decorator_on_class_is_currently_unsupported() {
		// Captures current-behavior: the frontend raises
		// UnsupportedStatementException on class-level decorators. When a
		// future chunk adds class-decorator support, flip this test to
		// assertThat(...).isNotNull() — it is the safety-net signal that
		// behavior actually changed.
		assertThatThrownBy(() -> FrontendTestSupport.parseSnippet("@annotated\nclass C:\n    pass\n"))
				.hasMessageContaining("");
	}
}
