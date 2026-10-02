package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import it.unive.pylisa.frontend.DiagnosticReporter.DiagnosticEvent;
import it.unive.pylisa.frontend.DiagnosticReporter.Severity;
import java.nio.file.Path;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer C: regression-guards for every currently-emitted unsoundness category.
 * <p>
 * Each test pins a category in two ways:
 * <ol>
 * <li>Log-based: via {@link LoggerCaptor}, confirms the diagnostic still
 * surfaces through log4j so users / existing tooling keep seeing it.</li>
 * <li>Reporter-based: via {@link DiagnosticReporter#events()}, confirms the
 * structured event is recorded with {@code Severity.UNSOUND} and the expected
 * feature label.</li>
 * </ol>
 * If a new unsoundness site is introduced in the frontend, add a case here so
 * the safety net stays exhaustive.
 */
final class UnsoundnessCatalogTest {

	private static final Logger LOG = LogManager.getLogger(UnsoundnessCatalogTest.class);

	private static final String FRONTEND_LOGGER = "it.unive.pylisa.frontend";

	@Test
	void async_def_is_treated_as_def_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend("async def f():\n    pass\n");
			LOG.debug("warnings: {}", cap.warnings());
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("async"));
			assertUnsoundFeaturesContain(pf.frontend().reporter().events(), "async");
		}
	}

	@Test
	void async_statement_is_treated_as_sync_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend(
					"async def f():\n    async for x in agen():\n        y = x\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("async"));
			assertUnsoundFeaturesContain(pf.frontend().reporter().events(), "async");
		}
	}

	@Test
	void await_expression_is_stripped_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend(
					"async def f():\n    x = await other()\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("await"));
			assertUnsoundFeaturesContain(pf.frontend().reporter().events(), "await");
		}
	}

	@Test
	void multiple_return_values_emit_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend("def f():\n    return 1, 2\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("return"));
			List<DiagnosticEvent> unsound = pf.frontend().reporter().events().stream()
					.filter(e -> e.severity() == Severity.UNSOUND)
					.toList();
			assertThat(unsound).isNotEmpty();
		}
	}

	@Test
	void try_except_is_modeled_as_bypass_with_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend(
					"def f():\n    try:\n        x = 1\n    except Exception:\n        x = 0\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("except")
					|| s.toLowerCase().contains("try"));
			List<DiagnosticEvent> unsound = pf.frontend().reporter().events().stream()
					.filter(e -> e.severity() == Severity.UNSOUND)
					.toList();
			assertThat(unsound).isNotEmpty();
		}
	}

	@Test
	void multiple_subscripts_emit_warning() throws Exception {
		try (LoggerCaptor cap = LoggerCaptor.forPackage(FRONTEND_LOGGER)) {
			var pf = FrontendTestSupport.parseSnippetWithFrontend("def f(m):\n    x = m[1, 2]\n");
			assertThat(cap.warnings()).anyMatch(s -> s.toLowerCase().contains("subscript"));
			assertUnsoundFeaturesContain(pf.frontend().reporter().events(), "multiple");
		}
	}

	@Test
	void strict_mode_promotes_async_unsoundness_to_failure() throws Exception {
		Path tmp = FrontendTestSupport.writeTempSnippet("async def f():\n    pass\n");
		assertThatThrownBy(() -> PyFrontend.strict(tmp.toString()).toLiSAProgram(false))
				.isInstanceOf(DiagnosticReporter.StrictModeViolation.class)
				.hasMessageContaining("async");
	}

	@Test
	void non_strict_mode_records_events_without_throwing() throws Exception {
		var pf = FrontendTestSupport.parseSnippetWithFrontend("async def f():\n    pass\n");
		assertThat(pf.frontend().strict()).isFalse();
		assertThat(pf.frontend().reporter().countBy(Severity.UNSOUND)).isGreaterThan(0);
	}

	/**
	 * Asserts the reporter recorded at least one {@code UNSOUND} event whose
	 * feature label matches the expected category (substring match, so tests
	 * survive minor label tweaks).
	 */
	private static void assertUnsoundFeaturesContain(
			List<DiagnosticEvent> events,
			String expectedFeature) {
		assertThat(events)
				.filteredOn(e -> e.severity() == Severity.UNSOUND)
				.extracting(DiagnosticEvent::feature)
				.anyMatch(f -> f.toLowerCase().contains(expectedFeature.toLowerCase()));
	}
}
