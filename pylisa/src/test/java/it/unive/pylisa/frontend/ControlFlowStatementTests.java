package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: control-flow statement visitors — if/elif/else, while, for (in list
 * / range / dict / for-else), try / except / finally, with.
 */
final class ControlFlowStatementTests {

	private static final Logger LOG = LogManager.getLogger(ControlFlowStatementTests.class);

	@Test
	void if_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("if x:\n    y = 1\n").program()).isNotNull();
	}

	@Test
	void if_else_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("if x:\n    y = 1\nelse:\n    y = 2\n").program()).isNotNull();
	}

	@Test
	void if_elif_else_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"if x:\n    y = 1\nelif x == 2:\n    y = 2\nelse:\n    y = 3\n").program()).isNotNull();
	}

	@Test
	void while_loop_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("while x > 0:\n    x = x - 1\n");
		LOG.debug("while parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void while_with_break_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("while True:\n    break\n").program()).isNotNull();
	}

	@Test
	void while_with_continue_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("while x:\n    continue\n").program()).isNotNull();
	}

	@Test
	void for_in_list_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("for i in [1, 2, 3]:\n    x = i\n").program()).isNotNull();
	}

	@Test
	void for_in_range_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("for i in range(10):\n    x = i\n").program()).isNotNull();
	}

	@Test
	void for_in_dict_items_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("d = {}\nfor k, v in d.items():\n    x = v\n").program())
				.isNotNull();
	}

	@Test
	void for_else_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"for i in range(5):\n    x = i\nelse:\n    y = 0\n").program()).isNotNull();
	}

	@Test
	void try_except_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"try:\n    x = 1\nexcept Exception:\n    x = 0\n").program()).isNotNull();
	}

	@Test
	void try_except_finally_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"try:\n    x = 1\nexcept ValueError as e:\n    x = 0\nfinally:\n    y = 2\n").program()).isNotNull();
	}

	@Test
	void with_statement_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("with open('x') as f:\n    y = f\n").program()).isNotNull();
	}

	@Test
	void with_multiple_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"with open('a') as a, open('b') as b:\n    pass\n").program()).isNotNull();
	}
}
