package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: attribute access, call, subscript, super(), and chained trailer
 * combinations — the branches inside {@code visitAtom_expr}.
 */
final class AccessExpressionTests {

	private static final Logger LOG = LogManager.getLogger(AccessExpressionTests.class);

	@Test
	void attribute_access_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a.b\n").program()).isNotNull();
	}

	@Test
	void chained_attribute_access_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a.b.c.d\n").program()).isNotNull();
	}

	@Test
	void simple_call_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = f()\n").program()).isNotNull();
	}

	@Test
	void call_with_positional_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = f(1, 2, 3)\n").program()).isNotNull();
	}

	@Test
	void call_with_keyword_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = f(a=1, b=2)\n").program()).isNotNull();
	}

	@Test
	void subscript_indexing_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a[0]\n").program()).isNotNull();
	}

	@Test
	void subscript_slicing_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a[1:5]\n").program()).isNotNull();
	}

	@Test
	void subscript_with_step_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a[1:5:2]\n").program()).isNotNull();
	}

	@Test
	void subscript_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("d = {}\nd[k] = v\n").program()).isNotNull();
	}

	@Test
	void chained_trailers_a_b_c_d_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a.b().c[0].d\n").program()).isNotNull();
	}

	@Test
	void super_call_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"class A:\n    def f(self):\n        super().__init__()\n");
		LOG.debug("super() parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void super_with_args_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class A:\n    def f(self):\n        super(A, self).__init__()\n").program())
						.isNotNull();
	}
}
