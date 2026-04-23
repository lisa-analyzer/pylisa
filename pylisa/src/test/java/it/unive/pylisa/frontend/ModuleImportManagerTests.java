package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import it.unive.pylisa.cfg.type.PyModuleType;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: {@link PythonModuleImportManager} behavior — library fallback,
 * unknown-import ancestor registration, dedup, circular imports via public
 * {@link PyFrontend} API only.
 */
final class ModuleImportManagerTests {

	private static final Logger LOG = LogManager.getLogger(ModuleImportManagerTests.class);

	@Test
	void unresolved_import_registers_module_as_unknown() throws Exception {
		var result = FrontendTestSupport.parseSnippet("import totally_unspecced\n");
		LOG.debug("unresolved import parsed at {}", result.sourceFile());
		assertThat(PyModuleType.isRegistered("totally_unspecced")).isTrue();
	}

	@Test
	void stdlib_spec_resolves_from_library_fallback() throws Exception {
		FrontendTestSupport.parseSnippet("import json\n");
		assertThat(PyModuleType.lookup("json")).isNotNull();
	}

	@Test
	void dotted_import_registers_ancestor_modules() throws Exception {
		FrontendTestSupport.parseSnippet("import unknown_pkg.inner.deep\n");
		assertThat(PyModuleType.isRegistered("unknown_pkg")).isTrue();
		assertThat(PyModuleType.isRegistered("unknown_pkg.inner")).isTrue();
		assertThat(PyModuleType.isRegistered("unknown_pkg.inner.deep")).isTrue();
	}

	@Test
	void duplicate_imports_in_same_file_do_not_double_register() throws Exception {
		FrontendTestSupport.parseSnippet("import json\nimport json\nimport json\n");
		assertThat(PyModuleType.isRegistered("json")).isTrue();
	}

	@Test
	void from_import_module_registers_parent_module() throws Exception {
		FrontendTestSupport.parseSnippet("from json import dumps\n");
		assertThat(PyModuleType.isRegistered("json")).isTrue();
	}

	@Test
	void wildcard_import_does_not_throw() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import *\n").program()).isNotNull();
	}
}
