package it.unive.pylisa.frontend;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeMember;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Shared test utilities for the {@code it.unive.pylisa.frontend} test package.
 * <p>
 * Tests exercise the frontend exclusively through its public API
 * ({@link PyFrontend}), never by calling visitor methods directly. Snippets are
 * materialised into a temp file so {@link PyFrontend}'s file-based constructors
 * can consume them unchanged.
 * <p>
 * <strong>TODO — strengthen Layer A assertions.</strong> Most current Layer A
 * tests only verify that the parse produced a non-null {@code Program}. A
 * future pass should turn these into structural assertions on the emitted CFG —
 * e.g., for {@code "x = 3"}, assert that the program contains a
 * {@code PyAssign} whose target is {@code x} and whose value is the integer
 * literal {@code 3}. Add the relevant CFG-introspection helpers here when the
 * assertions are tightened.
 */
public final class FrontendTestSupport {

	private static final Logger LOG = LogManager.getLogger(FrontendTestSupport.class);

	private FrontendTestSupport() {
	}

	/**
	 * Result of parsing a Python source through {@link PyFrontend}.
	 *
	 * @param program    the produced LiSA program
	 * @param sourceFile the file that backed the parse (temp file for snippets)
	 */
	public record ParseResult(
			Program program,
			Path sourceFile) {
	}

	/**
	 * Parses a Python snippet and returns the produced LiSA {@link Program}.
	 * The snippet is written to a temp file (deleted on JVM exit) so the
	 * {@link PyFrontend} public API stays the only entry point exercised.
	 */
	public static ParseResult parseSnippet(
			String snippet)
			throws Exception {
		Objects.requireNonNull(snippet, "snippet");
		Path tmp = Files.createTempFile("pylisa-test-", ".py");
		tmp.toFile().deleteOnExit();
		Files.writeString(tmp, snippet, StandardOpenOption.TRUNCATE_EXISTING);
		LOG.debug("parsing snippet at {} ({} chars)", tmp, snippet.length());
		PyFrontend fe = new PyFrontend(tmp.toString(), false);
		return new ParseResult(fe.toLiSAProgram(false), tmp);
	}

	/**
	 * Parses an existing Python file on disk and returns the produced LiSA
	 * {@link Program}.
	 */
	public static ParseResult parseFile(
			Path file)
			throws IOException {
		Objects.requireNonNull(file, "file");
		LOG.info("parsing test file {}", file);
		PyFrontend fe = new PyFrontend(file.toString(), false);
		try {
			return new ParseResult(fe.toLiSAProgram(false), file);
		} catch (Exception ex) {
			throw new IOException("parsing " + file + " failed", ex);
		}
	}

	/**
	 * Returns {@code true} iff a unit with the given fully-qualified name
	 * exists in the program.
	 */
	public static boolean hasUnit(
			Program program,
			String fqName) {
		return program.getUnit(fqName) != null;
	}

	/**
	 * Returns the first top-level {@link CodeMember} whose name matches, or
	 * {@link Optional#empty()} if none was produced.
	 */
	public static Optional<CodeMember> findCodeMember(
			Program program,
			String name) {
		for (Unit u : program.getUnits())
			for (CodeMember cm : u.getCodeMembers())
				if (cm.getDescriptor().getName().equals(name))
					return Optional.of(cm);
		return Optional.empty();
	}

	/**
	 * Returns the list of all top-level CodeMember names emitted by the
	 * frontend, sorted alphabetically for stable test assertions.
	 */
	public static List<String> allCodeMemberNames(
			Program program) {
		return program.getCodeMembersRecursively().stream()
				.map(cm -> cm.getDescriptor().getName())
				.sorted()
				.toList();
	}

	/**
	 * Returns the list of all unit names emitted by the frontend, sorted
	 * alphabetically for stable test assertions.
	 */
	public static List<String> allUnitNames(
			Program program) {
		return program.getUnits().stream()
				.map(Unit::getName)
				.sorted()
				.toList();
	}
}
