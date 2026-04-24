package it.unive.pylisa.frontend;

import it.unive.lisa.program.SourceCodeLocation;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Structured diagnostic sink for the PyLiSA front-end. Introduced in Chunk 6 of
 * the front-end refactor to replace the scattered {@code unsound} and
 * {@code unsupported} helpers.
 * <p>
 * Events are both recorded in memory (for programmatic inspection via
 * {@link #events()}) and mirrored to log4j at the appropriate level so tests
 * keep working with {@code LoggerCaptor} and users continue to see them in
 * console output.
 * <p>
 * When constructed in {@code strict} mode, an {@link Severity#UNSOUND} event is
 * escalated to a {@link StrictModeViolation} after being recorded and logged.
 * {@link Severity#UNSUPPORTED} events are <em>not</em> auto-thrown — the caller
 * is expected to follow the
 * {@link #report(Severity, SourceCodeLocation, String, String)} call with its
 * own control-flow signal (e.g. throwing
 * {@link it.unive.pylisa.UnsupportedStatementException}).
 */
public final class DiagnosticReporter {

	private static final Logger LOG = LogManager.getLogger(DiagnosticReporter.class);

	/** Severity of a single diagnostic event. */
	public enum Severity {
		/** Informational: benign observation, no impact on soundness. */
		INFO,
		/**
		 * Unsound: the frontend approximated or dropped a construct in a way
		 * that may affect analysis correctness.
		 */
		UNSOUND,
		/** Unsupported: the grammar feature is not yet implemented. */
		UNSUPPORTED
	}

	/**
	 * Immutable record of a single diagnostic occurrence.
	 *
	 * @param severity the severity category
	 * @param location where in the source the event originated (may be
	 *                     {@code null} for events outside any ANTLR context)
	 * @param feature  a short, user-facing feature label (e.g. "async",
	 *                     "return", "subscript"); used for categorical
	 *                     filtering
	 * @param detail   the full human-readable message
	 */
	public record DiagnosticEvent(
			Severity severity,
			SourceCodeLocation location,
			String feature,
			String detail) {
		public DiagnosticEvent {
			Objects.requireNonNull(severity, "severity");
			Objects.requireNonNull(feature, "feature");
			Objects.requireNonNull(detail, "detail");
		}
	}

	/**
	 * Thrown from {@link #report(Severity, SourceCodeLocation, String, String)}
	 * when the reporter is in strict mode and an {@link Severity#UNSOUND} event
	 * is recorded.
	 */
	public static final class StrictModeViolation extends RuntimeException {

		private static final long serialVersionUID = 1L;

		private final DiagnosticEvent event;

		StrictModeViolation(
				DiagnosticEvent e) {
			super("[strict] " + e.feature() + " at " + e.location() + ": " + e.detail());
			this.event = e;
		}

		public DiagnosticEvent event() {
			return event;
		}
	}

	private final List<DiagnosticEvent> events = new ArrayList<>();
	private final boolean strict;

	public DiagnosticReporter(
			boolean strict) {
		this.strict = strict;
	}

	/**
	 * Records a diagnostic event, mirrors it to log4j at the appropriate level,
	 * and — in strict mode — throws {@link StrictModeViolation} for
	 * {@link Severity#UNSOUND} events.
	 *
	 * @param sev     severity category
	 * @param at      source location of the construct that triggered the event
	 *                    (may be {@code null})
	 * @param feature short feature label for categorical filtering
	 * @param detail  full human-readable message
	 *
	 * @return the recorded event
	 */
	public DiagnosticEvent report(
			Severity sev,
			SourceCodeLocation at,
			String feature,
			String detail) {
		DiagnosticEvent e = new DiagnosticEvent(sev, at, feature, detail);
		events.add(e);
		switch (sev) {
		case INFO -> LOG.info("[{}] {} at {}: {}", sev, feature, at, detail);
		case UNSOUND -> LOG.warn("[{}] {} at {}: {}", sev, feature, at, detail);
		case UNSUPPORTED -> LOG.error("[{}] {} at {}: {}", sev, feature, at, detail);
		}
		if (strict && sev == Severity.UNSOUND)
			throw new StrictModeViolation(e);
		return e;
	}

	/**
	 * Returns an unmodifiable snapshot of all events recorded so far, in the
	 * order they were reported.
	 */
	public List<DiagnosticEvent> events() {
		return List.copyOf(events);
	}

	/** Returns the count of events of the given severity. */
	public long countBy(
			Severity sev) {
		return events.stream().filter(e -> e.severity() == sev).count();
	}

	/** Returns {@code true} iff the reporter is in strict mode. */
	public boolean strict() {
		return strict;
	}
}
