package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * White-box cases stressing {@code visitImport_from} — Chunk 4 flattens its
 * class/module/member classification logic. Targets every relative-dot depth,
 * explicit-name lists, and the wildcard form.
 */
final class VisitImportFromTargetedTests {

	private static final Logger LOG = LogManager.getLogger(VisitImportFromTargetedTests.class);

	@Test
	void from_dot_import_single_name_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("from . import sibling\n");
		LOG.debug("relative 1-dot at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void from_dot_dot_import_single_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from .. import parent\n").program()).isNotNull();
	}

	@Test
	void from_dot_dot_dot_import_single_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from ... import grand\n").program()).isNotNull();
	}

	@Test
	void from_module_import_multiple_names_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import dumps, loads, JSONDecoder\n").program())
				.isNotNull();
	}

	@Test
	void from_module_import_wildcard_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import *\n").program()).isNotNull();
	}

	@Test
	void from_dotted_module_import_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from os.path import join\n").program()).isNotNull();
	}

	@Test
	void from_module_import_aliased_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import dumps as d, loads as l\n").program())
				.isNotNull();
	}
}
