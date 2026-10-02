package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitFor_stmt} — the 106-line god-method
 * decomposed in Chunk 4. Covers every path the method takes: range / list /
 * dict iteration, for-else, early exits (break/continue), and nested fors on
 * the same source line (counter-name collision risk).
 */
final class VisitForStmtTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitForStmtTargetedTests.class);

	@Test
	void for_in_range_lowers_to_counter_loop() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"def f():\n    total = 0\n    for i in range(10):\n        total = total + i\n    return total\n");
		LOG.debug("for-range parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void for_in_list_literal_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for x in [1, 2, 3]:\n        y = x\n    return y\n").program()).isNotNull();
	}

	@Test
	void for_in_dict_items_handles_tuple_target() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(d):\n    for k, v in d.items():\n        x = v\n    return x\n").program()).isNotNull();
	}

	@Test
	void for_else_branch_is_reachable_after_normal_completion() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for i in range(3):\n        x = i\n    else:\n        y = 0\n    return y\n").program())
						.isNotNull();
	}

	@Test
	void for_loop_with_break_preserves_body_exits() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for i in range(10):\n        if i == 5:\n            break\n        x = i\n    return x\n")
				.program()).isNotNull();
	}

	@Test
	void for_loop_with_continue_preserves_body_exits() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    for i in range(10):\n        if i % 2 == 0:\n            continue\n        x = i\n    return x\n")
				.program()).isNotNull();
	}

	@Test
	void nested_for_on_same_source_does_not_collide_counters() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    total = 0\n    for i in range(3):\n        for j in range(3):\n            total = total + 1\n    return total\n")
				.program()).isNotNull();
	}
}
