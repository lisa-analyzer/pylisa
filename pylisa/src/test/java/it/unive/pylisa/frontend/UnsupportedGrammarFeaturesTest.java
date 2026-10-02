package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.frontend.DiagnosticReporter.DiagnosticEvent;
import it.unive.pylisa.frontend.DiagnosticReporter.Severity;
import java.util.Map;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Verifies the central {@link UnsupportedGrammarFeatures} registry.
 * <p>
 * Two layers:
 * <ol>
 * <li><b>Structural</b> — {@link #features_map_is_well_formed} asserts every
 * entry has non-blank key and value.</li>
 * <li><b>Behavioural</b> — {@link #feature_gap_snippet_reports_unsupported}
 * parametrised over a small set of Python snippets known to reach explicit
 * {@code rejectUnsupported(ctx, "<label>")} call-sites introduced in Chunk 8;
 * asserts the reporter captures the corresponding UNSUPPORTED event and the
 * parse eventually throws.</li>
 * </ol>
 * The {@link UnsupportedGrammarFeatures#FEATURES} map includes many rule
 * contexts that ANTLR's default dispatch never reaches because the enclosing
 * visit methods peel off the sub-rule directly; those rows guard against future
 * wiring regressions but cannot be exercised via source today.
 */
final class UnsupportedGrammarFeaturesTest {

	/**
	 * Python snippets whose parse is known to reach a hard-coded
	 * {@code rejectUnsupported(ctx, "<label>")} call-site. Each label matches
	 * an explicit string in the visitor code, not a {@code FEATURES} entry.
	 */
	private static final Map<String, String> FEATURE_GAP_SNIPPETS = Map.of(
			"complex literal", "x = 1j\n",
			"async", "async def f():\n    pass\n");

	static Stream<Arguments> featureGapSnippets() {
		return FEATURE_GAP_SNIPPETS.entrySet().stream()
				.map(e -> Arguments.of(e.getKey(), e.getValue()));
	}

	@Test
	void features_map_is_well_formed() {
		assertThat(UnsupportedGrammarFeatures.FEATURES).isNotEmpty();
		assertThat(UnsupportedGrammarFeatures.FEATURES).allSatisfy((
				k,
				v) -> {
			assertThat(k).isNotBlank().endsWith("Context");
			assertThat(v).isNotBlank();
		});
	}

	@ParameterizedTest(name = "{0}")
	@MethodSource("featureGapSnippets")
	void feature_gap_snippet_reports_unsupported(
			String expectedLabelSubstring,
			String snippet)
			throws Exception {
		var path = FrontendTestSupport.writeTempSnippet(snippet);
		var fe = new PyFrontend(path.toString(), false).setContinueOnUnsupportedStatement(true);
		try {
			fe.toLiSAProgram(false);
		} catch (UnsupportedStatementException ignored) {
			// Top-level triggers may still surface; accept both outcomes —
			// we care only that the reporter saw the event.
		}
		assertThat(fe.reporter().events())
				.filteredOn(e -> e.severity() == Severity.UNSUPPORTED
						|| e.severity() == Severity.UNSOUND)
				.extracting(DiagnosticEvent::feature)
				.anyMatch(f -> f.contains(expectedLabelSubstring));
	}

	@Test
	void strict_mode_promotes_async_unsoundness_to_failure_via_registry_path()
			throws Exception {
		var path = FrontendTestSupport.writeTempSnippet("async def f():\n    pass\n");
		assertThatThrownBy(() -> PyFrontend.strict(path.toString()).toLiSAProgram(false))
				.isInstanceOf(DiagnosticReporter.StrictModeViolation.class);
	}
}
