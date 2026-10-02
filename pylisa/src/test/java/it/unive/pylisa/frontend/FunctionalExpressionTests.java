package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: conditional (ternary) and lambda expression forms.
 */
final class FunctionalExpressionTests {

	private static final Logger LOG = LogManager.getLogger(FunctionalExpressionTests.class);

	@Test
	void ternary_expression_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("x = a if cond else b\n");
		LOG.debug("ternary parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void nested_ternary_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a if c1 else (b if c2 else c)\n").program()).isNotNull();
	}

	@Test
	void lambda_no_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("f = lambda: 42\n").program()).isNotNull();
	}

	@Test
	void lambda_single_arg_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("f = lambda x: x + 1\n").program()).isNotNull();
	}

	@Test
	void lambda_multi_arg_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("f = lambda x, y: x + y\n").program()).isNotNull();
	}

	@Test
	void lambda_with_default_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("f = lambda x, y=2: x + y\n").program()).isNotNull();
	}
}
