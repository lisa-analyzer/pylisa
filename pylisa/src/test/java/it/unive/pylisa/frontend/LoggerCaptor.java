package it.unive.pylisa.frontend;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.core.LogEvent;
import org.apache.logging.log4j.core.LoggerContext;
import org.apache.logging.log4j.core.appender.AbstractAppender;
import org.apache.logging.log4j.core.config.Configuration;
import org.apache.logging.log4j.core.config.LoggerConfig;

/**
 * Captures log4j2 events for a specific logger for the duration of a test.
 * <p>
 * Usage (always with try-with-resources):
 *
 * <pre>
 * try (LoggerCaptor cap = LoggerCaptor.forPackage("it.unive.pylisa.frontend")) {
 * 	// ... code that emits logs ...
 * 	assertThat(cap.warnings()).anyMatch(s -&gt; s.contains("async"));
 * }
 * </pre>
 *
 * Assertions MUST be keyword-based (not exact-text) so they survive later
 * rewording of diagnostic messages.
 */
public final class LoggerCaptor implements AutoCloseable {

	private final LoggerConfig loggerConfig;
	private final CollectingAppender appender;
	private final Level previousLevel;
	private final LoggerContext ctx;

	/**
	 * Attaches a captor to a logger identified by name (usually a package).
	 */
	public static LoggerCaptor forPackage(
			String loggerName) {
		return new LoggerCaptor(loggerName);
	}

	/**
	 * Attaches a captor to the logger for the given class.
	 */
	public static LoggerCaptor forClass(
			Class<?> c) {
		return new LoggerCaptor(c.getName());
	}

	private LoggerCaptor(
			String loggerName) {
		this.ctx = (LoggerContext) LogManager.getContext(false);
		Configuration config = ctx.getConfiguration();
		this.loggerConfig = config.getLoggerConfig(loggerName);
		this.previousLevel = loggerConfig.getLevel();
		this.appender = new CollectingAppender("LoggerCaptor-" + System.nanoTime());
		appender.start();
		loggerConfig.addAppender(appender, Level.ALL, null);
		// Widen the level so DEBUG/TRACE events reach us if the test needs
		// them; tests that only care about WARN/ERROR simply ignore the rest.
		loggerConfig.setLevel(Level.TRACE);
		ctx.updateLoggers();
	}

	/** All formatted log messages, in emission order. */
	public List<String> all() {
		return appender.get(null);
	}

	/** Formatted messages emitted at WARN level. */
	public List<String> warnings() {
		return appender.get(Level.WARN);
	}

	/** Formatted messages emitted at ERROR level. */
	public List<String> errors() {
		return appender.get(Level.ERROR);
	}

	/** Formatted messages emitted at INFO level. */
	public List<String> infos() {
		return appender.get(Level.INFO);
	}

	/** Formatted messages emitted at DEBUG level. */
	public List<String> debugs() {
		return appender.get(Level.DEBUG);
	}

	@Override
	public void close() {
		loggerConfig.removeAppender(appender.getName());
		loggerConfig.setLevel(previousLevel);
		appender.stop();
		ctx.updateLoggers();
	}

	private static final class CollectingAppender extends AbstractAppender {

		private final List<LogEvent> events = new CopyOnWriteArrayList<>();

		CollectingAppender(
				String name) {
			super(name, null, null, false, null);
		}

		@Override
		public void append(
				LogEvent event) {
			events.add(event.toImmutable());
		}

		List<String> get(
				Level filter) {
			List<String> out = new ArrayList<>();
			for (LogEvent e : events)
				if (filter == null || e.getLevel().equals(filter))
					out.add(e.getMessage().getFormattedMessage());
			return Collections.unmodifiableList(out);
		}
	}
}
