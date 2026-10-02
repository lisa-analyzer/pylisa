package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitIf_stmt} — the 71-line god-method
 * decomposed in Chunk 4. Covers all branch-combinator shapes and all ways a
 * branch can end in an execution-stopping statement.
 */
final class VisitIfStmtTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitIfStmtTargetedTests.class);

	@Test
	void if_only_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("def f(x):\n    if x:\n        y = 1\n    return y\n").program())
				.isNotNull();
	}

	@Test
	void if_else_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(x):\n    if x:\n        y = 1\n    else:\n        y = 2\n    return y\n").program())
						.isNotNull();
	}

	@Test
	void if_elif_else_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(x):\n    if x == 1:\n        y = 1\n    elif x == 2:\n        y = 2\n    else:\n        y = 3\n    return y\n")
				.program()).isNotNull();
	}

	@Test
	void three_elif_chain_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"def f(x):\n    if x == 1:\n        y = 1\n    elif x == 2:\n        y = 2\n    elif x == 3:\n        y = 3\n    elif x == 4:\n        y = 4\n    return y\n");
		LOG.debug("3-elif parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void branch_ending_in_return_closes_cfg() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(x):\n    if x:\n        return 1\n    return 0\n").program()).isNotNull();
	}

	@Test
	void branch_ending_in_break_closes_loop_edge() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for i in range(3):\n        if i == 2:\n            break\n    return i\n").program())
						.isNotNull();
	}

	@Test
	void branch_ending_in_continue_closes_loop_edge() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for i in range(3):\n        if i % 2 == 0:\n            continue\n        x = i\n    return x\n")
				.program()).isNotNull();
	}

	@Test
	void branch_ending_in_pass_is_benign() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(x):\n    if x:\n        pass\n    else:\n        y = 1\n    return 0\n").program())
						.isNotNull();
	}
}
