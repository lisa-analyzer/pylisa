package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: class-definition shape coverage — empty, single-inherit,
 * multi-inherit, nested, methods + fields.
 */
final class ClassDefinitionTests {

	private static final Logger LOG = LogManager.getLogger(ClassDefinitionTests.class);

	@Test
	void empty_class_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("class A:\n    pass\n");
		LOG.debug("empty class parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void class_with_single_inherit_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("class B(A):\n    pass\n").program()).isNotNull();
	}

	@Test
	void class_with_multi_inherit_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("class C(A, B):\n    pass\n").program()).isNotNull();
	}

	@Test
	void nested_class_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("class Outer:\n    class Inner:\n        pass\n").program())
				.isNotNull();
	}

	@Test
	void class_with_methods_and_fields_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"class C:\n    x = 1\n    def f(self):\n        return self.x\n").program()).isNotNull();
	}

	@Test
	void two_classes_with_same_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(
				"if c:\n    class Secret:\n        pass\nelse:\n    class Secret:\n        pass\n").program())
						.isNotNull();
	}
}
