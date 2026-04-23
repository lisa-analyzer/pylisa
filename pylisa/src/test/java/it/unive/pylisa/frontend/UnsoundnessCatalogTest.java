package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer C: regression-guards for every currently-emitted unsoundness category.
 * <p>
 * Assertions are <strong>keyword-based</strong> rather than exact-text matches
 * because Chunk 6 rewrites the diagnostic emission path. Each test pins the
 * <em>category</em> of warning, not its wording.
 * <p>
 * If a new unsoundness site is introduced in the frontend, add a case here so
 * the safety net stays exhaustive.
 */
final class UnsoundnessCatalogTest {

	private static final Logger LOG = LogManager.getLogger(UnsoundnessCatalogTest.class);

	private static final String FRONTEND_LOGGER = "it.unive.pylisa.frontend.PyFrontendBase";

	@Test
	void async_def_is_treated_as_def_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet("async def f():\n    pass\n");
			LOG.debug("warnings: {}", cap.warnings());
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("async"));
		}
	}

	@Test
	void async_statement_is_treated_as_sync_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet(
					"async def f():\n    async for x in agen():\n        y = x\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("async"));
		}
	}

	@Test
	void await_expression_is_stripped_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet("async def f():\n    x = await other()\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("await"));
		}
	}

	@Test
	void multiple_return_values_emit_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet("def f():\n    return 1, 2\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("return"));
		}
	}

	@Test
	void try_except_is_modeled_as_bypass_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet(
					"def f():\n    try:\n        x = 1\n    except Exception:\n        x = 0\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("except")
					|| s.toLowerCase().contains("try"));
		}
	}

	@Test
	void multiple_subscripts_emit_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			FrontendTestSupport.parseSnippet("def f(m):\n    x = m[1, 2]\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("subscript"));
		}
	}
}
