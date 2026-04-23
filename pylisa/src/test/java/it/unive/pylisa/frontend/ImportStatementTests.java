package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: every import-statement shape — plain import, aliased, dotted,
 * from-import (name / wildcard / relative).
 */
final class ImportStatementTests {

	private static final Logger LOG = LogManager.getLogger(ImportStatementTests.class);

	@Test
	void import_single_module_parses() throws Exception {
		var result = FrontendTestSupport.parseSnippet("import json\n");
		LOG.debug("import parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void import_with_alias_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("import json as j\n").program()).isNotNull();
	}

	@Test
	void import_dotted_module_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("import os.path\n").program()).isNotNull();
	}

	@Test
	void import_multiple_modules_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("import os, sys\n").program()).isNotNull();
	}

	@Test
	void from_import_single_name_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import dumps\n").program()).isNotNull();
	}

	@Test
	void from_import_multiple_names_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import dumps, loads\n").program()).isNotNull();
	}

	@Test
	void from_import_wildcard_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import *\n").program()).isNotNull();
	}

	@Test
	void from_import_with_alias_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from json import dumps as d\n").program()).isNotNull();
	}

	@Test
	void relative_import_single_dot_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from . import sibling\n").program()).isNotNull();
	}

	@Test
	void relative_import_two_dots_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("from .. import parent\n").program()).isNotNull();
	}
}
