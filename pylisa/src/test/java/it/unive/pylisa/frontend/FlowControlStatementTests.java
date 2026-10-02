package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: return / break / continue / raise / yield — the statements that stop
 * or redirect CFG execution.
 */
final class FlowControlStatementTests {

	private static final Logger LOG = LogManager.getLogger(FlowControlStatementTests.class);

	@Test
	void return_without_value_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    return\n").program()).isNotNull();
	}

	@Test
	void return_with_value_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("def f():\n    return 42\n");
		LOG.debug("return parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void return_with_tuple_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    return 1, 2\n").program()).isNotNull();
	}

	@Test
	void raise_bare_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    raise\n").program()).isNotNull();
	}

	@Test
	void raise_exception_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    raise ValueError('x')\n").program()).isNotNull();
	}

	@Test
	void raise_from_cause_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    raise ValueError() from RuntimeError()\n")
				.program()).isNotNull();
	}

	@Test
	void yield_expression_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def g():\n    yield 1\n").program()).isNotNull();
	}

	@Test
	void yield_from_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def g():\n    yield from range(3)\n").program()).isNotNull();
	}
}
