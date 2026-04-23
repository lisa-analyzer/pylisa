package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: parameter-list shape coverage — typed, default, varargs, kwargs,
 * keyword-only (after {@code *}).
 */
final class ParameterTests {

	private static final Logger LOG = LogManager.getLogger(ParameterTests.class);

	@Test
	void typed_parameter_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("def f(a: int):\n    pass\n");
		LOG.debug("typed parses at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void defaulted_parameter_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(a=1):\n    pass\n").program()).isNotNull();
	}

	@Test
	void varargs_parameter_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(*args):\n    pass\n").program()).isNotNull();
	}

	@Test
	void kwargs_parameter_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(**kw):\n    pass\n").program()).isNotNull();
	}

	@Test
	void keyword_only_parameter_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(a, *, b):\n    pass\n").program()).isNotNull();
	}
}
