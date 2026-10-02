package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.pylisa.cfg.expression.PyAssign;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * Walrus operator ({@code :=}, PEP 572) parser and desugaring tests. The
 * front-end has no dedicated walrus CFG node — {@code visitNamedexpr_test}
 * desugars the walrus into a plain {@link PyAssign} that the control-flow
 * visitor splices into the block before the guard. These tests pin the
 * desugaring contract so later refactors can't silently drop it.
 */
final class WalrusOperatorTests {

	private static Program parseFixture(
			String name)
			throws Exception {
		return FrontendTestSupport.parseFile(Path.of("py-testcases/walrus", name)).program();
	}

	private static long countPyAssigns(
			Program program) {
		long total = 0;
		for (Unit u : program.getUnits())
			for (CodeMember cm : u.getCodeMembers())
				if (cm instanceof CFG cfg)
					total += cfg.getNodeList().getNodes().stream().filter(n -> n instanceof PyAssign).count();
		return total;
	}

	/**
	 * Compare PyAssign count against a walrus-free baseline to isolate the
	 * walrus-induced preludes from the stdlib's own assignments.
	 */
	private static long walrusPreludeCount(
			String walrusSnippet,
			String baselineSnippet)
			throws Exception {
		long withWalrus = countPyAssigns(FrontendTestSupport.parseSnippet(walrusSnippet).program());
		long baseline = countPyAssigns(FrontendTestSupport.parseSnippet(baselineSnippet).program());
		return withWalrus - baseline;
	}

	@Test
	void simple_if_walrus_parses_and_desugars_to_PyAssign() throws Exception {
		Program program = parseFixture("simple_if.py");
		assertThat(program).isNotNull();
		long delta = walrusPreludeCount(
				"def go():\n    if (x := 5):\n        return x\n    return 0\n",
				"def go():\n    if 5:\n        return 5\n    return 0\n");
		assertThat(delta).as("one walrus → one extra PyAssign vs baseline").isEqualTo(1L);
	}

	@Test
	void walrus_and_chain_parses() throws Exception {
		Program program = parseFixture("and_chain.py");
		assertThat(program).isNotNull();
		long delta = walrusPreludeCount(
				"def go():\n    if (a := 1) and (b := 2):\n        return a + b\n    return 0\n",
				"def go():\n    if 1 and 2:\n        return 1 + 2\n    return 0\n");
		assertThat(delta).as("two walruses → two extra PyAssigns vs baseline").isEqualTo(2L);
	}

	@Test
	void walrus_while_guard_parses() throws Exception {
		Program program = parseFixture("while_guard.py");
		assertThat(program).isNotNull();
		long delta = walrusPreludeCount(
				"def drain(q):\n    while (item := q.pop()):\n        handle(item)\n",
				"def drain(q):\n    while q.pop():\n        handle(q)\n");
		assertThat(delta).as("while walrus → one extra PyAssign vs baseline").isEqualTo(1L);
	}

	@Test
	void walrus_snippet_inline_if_parses() throws Exception {
		// Inline-smoke: exercises the same code path dispatch's 17
		// walrus-bearing files use — a guarded d.get(...) chain.
		Program program = FrontendTestSupport.parseSnippet(
				"def check(d):\n"
						+ "    if (v := d.get('x')):\n"
						+ "        return v\n"
						+ "    return None\n")
				.program();
		assertThat(program).isNotNull();
		long delta = walrusPreludeCount(
				"def check(d):\n    if (v := d.get('x')):\n        return v\n    return None\n",
				"def check(d):\n    if d.get('x'):\n        return d\n    return None\n");
		assertThat(delta).as("d.get-style walrus → one extra PyAssign vs baseline").isEqualTo(1L);
	}

	@Test
	void walrus_outside_supported_position_is_translated_unsoundly_and_reported() throws Exception {
		// PEP 572 allows (x := 5) as a parenthesised atom in expressions, but
		// without an active prelude frame the desugar has nowhere to hoist
		// the assignment: the binding is dropped, the value kept, and the
		// function is reported as translated unsoundly, never silently
		var parsed = FrontendTestSupport.parseSnippetWithFrontend(
				"def go():\n"
						+ "    y = (x := 7)\n"
						+ "    return x + y\n");
		assertThat(parsed.frontend().reporter().events())
				.anyMatch(ev -> ev.severity() == DiagnosticReporter.Severity.UNSOUND
						&& ev.detail().contains(":= walrus in unsupported position"));
	}
}
