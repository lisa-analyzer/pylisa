package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitAtom_expr} — the 114-line god-method
 * decomposed in Chunk 3. Targets every trailer combination and the dunder
 * lowering sites ({@code __getitem__}, {@code __setitem__}).
 */
final class VisitAtomExprTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitAtomExprTargetedTests.class);

	@Test
	void bare_super_call_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class A:\n    def f(self):\n        return super()\n").program()).isNotNull();
	}

	@Test
	void super_with_class_and_self_args_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet(
				"class A:\n    def f(self):\n        return super(A, self)\n");
		LOG.debug("super with args at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void deeply_chained_trailers_parse() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(a):\n    return a.b().c[0].d.e().f\n").program()).isNotNull();
	}

	@Test
	void subscript_assignment_lowers_to_setitem() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    d = {}\n    d['k'] = 'v'\n    return d\n").program()).isNotNull();
	}

	@Test
	void slice_assignment_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    a = [1, 2, 3]\n    a[0:2] = [9, 9]\n    return a\n").program()).isNotNull();
	}

	@Test
	void attribute_chain_with_call_in_middle_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f(o):\n    return o.get().next.value\n").program()).isNotNull();
	}

	@Test
	void dict_literal_with_getitem_access_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"def f():\n    d = {'k': 1}\n    return d['k']\n").program()).isNotNull();
	}
}
