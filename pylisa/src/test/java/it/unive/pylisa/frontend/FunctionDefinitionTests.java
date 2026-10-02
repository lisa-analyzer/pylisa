package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: {@code def} / {@code async def} shape coverage — args, kwargs,
 * defaults, type hints, nested def.
 */
final class FunctionDefinitionTests {

	private static final Logger LOG = LogManager.getLogger(FunctionDefinitionTests.class);

	@Test
	void def_no_args_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("def f():\n    pass\n");
		LOG.debug("def parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void def_with_positional_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(a, b, c):\n    pass\n").program()).isNotNull();
	}

	@Test
	void def_with_default_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(a, b=2):\n    pass\n").program()).isNotNull();
	}

	@Test
	void def_with_varargs_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(*args):\n    pass\n").program()).isNotNull();
	}

	@Test
	void def_with_kwargs_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(**kwargs):\n    pass\n").program()).isNotNull();
	}

	@Test
	void def_with_type_hints_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(a: int, b: str) -> bool:\n    return True\n").program())
				.isNotNull();
	}

	@Test
	void async_def_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("async def f():\n    pass\n").program()).isNotNull();
	}

	@Test
	void nested_def_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def outer():\n    def inner():\n        pass\n    return inner\n").program()).isNotNull();
	}
}
