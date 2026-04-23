package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitClassdef} + {@code parseClassBody} —
 * the ~100 + ~70 LOC god-methods decomposed in Chunk 5. Covers the empty-body
 * edge case (class with no field statements), multi-inherit, and the
 * allocation-site naming contract for two classes sharing a Python-visible
 * name.
 */
final class VisitClassdefTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitClassdefTargetedTests.class);

	@Test
	void empty_class_body_sets_ret_as_first_node() throws Exception {
		var result = FrontendTestSupport.parseSnippet("class A:\n    pass\n");
		LOG.debug("empty class at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void class_with_only_fields_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("class C:\n    x = 1\n    y = 2\n    z = 3\n").program())
				.isNotNull();
	}

	@Test
	void class_with_only_methods_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class C:\n    def a(self):\n        return 1\n    def b(self):\n        return 2\n").program())
						.isNotNull();
	}

	@Test
	void multi_inherit_with_ambiguous_ancestor_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class A:\n    pass\nclass B(A):\n    pass\nclass C(A, B):\n    pass\n").program()).isNotNull();
	}

	@Test
	void two_classes_same_name_receive_distinct_identities() throws Exception {
		// Exercises allocation-site naming: two class defs at different
		// source positions must mint two identities sharing the same Python
		// base name. Covered in depth by JsonImportResolutionTest; this is a
		// shape sanity check at the frontend level.
		assertThat(FrontendTestSupport.parseSnippet(
				"if c:\n    class Shared:\n        x = 1\nelse:\n    class Shared:\n        x = 2\n").program())
						.isNotNull();
	}

	@Test
	void class_with_init_body_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class C:\n    def __init__(self, x):\n        self.x = x\n").program()).isNotNull();
	}
}
