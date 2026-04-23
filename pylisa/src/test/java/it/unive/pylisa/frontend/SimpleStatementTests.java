package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: every "simple statement" branch of the grammar — assign / annassign
 * / augassign / del / pass / assert / global / nonlocal / async markers.
 */
final class SimpleStatementTests {

	private static final Logger LOG = LogManager.getLogger(SimpleStatementTests.class);

	@Test
	void simple_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1\n").program()).isNotNull();
	}

	@Test
	void multiple_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("a = b = 1\n").program()).isNotNull();
	}

	@Test
	void tuple_unpacking_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("a, b = 1, 2\n").program()).isNotNull();
	}

	@Test
	void annotated_assignment_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("x: int = 1\n");
		LOG.debug("annassign at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void augmented_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1\nx += 2\n").program()).isNotNull();
	}

	@Test
	void del_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1\ndel x\n").program()).isNotNull();
	}

	@Test
	void pass_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    pass\n").program()).isNotNull();
	}

	@Test
	void assert_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("assert x > 0\n").program()).isNotNull();
	}

	@Test
	void assert_with_message_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("assert x > 0, 'positive'\n").program()).isNotNull();
	}

	@Test
	void global_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f():\n    global x\n    x = 1\n").program()).isNotNull();
	}

	@Test
	void nonlocal_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def outer():\n    x = 1\n    def inner():\n        nonlocal x\n        x = 2\n").program())
						.isNotNull();
	}

	@Test
	void expression_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("f()\n").program()).isNotNull();
	}
}
