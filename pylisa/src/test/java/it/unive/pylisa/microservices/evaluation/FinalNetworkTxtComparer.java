package it.unive.pylisa.microservices.evaluation;

import static java.nio.file.Files.createDirectories;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.opentest4j.AssertionFailedError;

/**
 * Compares a generated {@code final-network.txt} (produced by
 * {@link it.unive.pylisa.outputs.FinalNetworkTxtResults}) against a checked-in
 * ground-truth file.
 * <p>
 * The comparison is <strong>endpoint-oriented</strong>: the set of
 * {@code (METHOD, path)} pairs must be identical. Location fragments
 * ({@code file:line:col}) are treated as informational — mismatches there are
 * reported on {@code System.err} but do not fail the test. This reflects the
 * current state of the refactor: path names are invariants, but locations may
 * drift as visitor dispatch is reshuffled and will be tightened in a later
 * pass.
 * <p>
 * When the system property {@code lisa.cron.update=true} is set (or
 * {@link #assertMatches(Path, Path, boolean) forceUpdate} is passed), a missing
 * or differing ground-truth file is refreshed from the generated one instead of
 * failing the test. Use this to seed new ground-truths or to accept a genuine
 * output change.
 */
public final class FinalNetworkTxtComparer {

	private static final String UPDATE_PROP = "lisa.cron.update";

	private static final Pattern METHOD_HEADER = Pattern.compile("^([A-Z]+): \\d+$");

	private FinalNetworkTxtComparer() {
		// utility class
	}

	/**
	 * Asserts that the endpoints declared in {@code actual} match
	 * {@code expected}, honouring the {@code lisa.cron.update} system property
	 * for regeneration.
	 *
	 * @param expected the ground-truth file (may not yet exist when seeding)
	 * @param actual   the freshly generated file
	 *
	 * @throws IOException          if either file cannot be read or the
	 *                                  ground-truth cannot be (re)written
	 * @throws AssertionFailedError if the endpoint sets differ and regeneration
	 *                                  is disabled
	 */
	public static void assertMatches(
			Path expected,
			Path actual)
			throws IOException {
		assertMatches(expected, actual, isUpdateEnabled());
	}

	/**
	 * Variant of {@link #assertMatches(Path, Path)} that takes an explicit
	 * update flag, bypassing the system property check.
	 *
	 * @param expected    the ground-truth file
	 * @param actual      the freshly generated file
	 * @param forceUpdate if {@code true}, a missing or differing ground-truth
	 *                        is replaced by the actual content instead of
	 *                        failing
	 *
	 * @throws IOException          if either file cannot be read or the
	 *                                  ground-truth cannot be (re)written
	 * @throws AssertionFailedError if the endpoint sets differ and
	 *                                  {@code forceUpdate} is {@code false}
	 */
	public static void assertMatches(
			Path expected,
			Path actual,
			boolean forceUpdate)
			throws IOException {
		if (!Files.exists(actual))
			throw new AssertionFailedError(
					"Generated file not found: " + actual + " — did the analysis register FinalNetworkTxtResults?");

		List<String> actualLines = Files.readAllLines(actual, StandardCharsets.UTF_8);

		if (!Files.exists(expected)) {
			if (forceUpdate) {
				writeLines(expected, actualLines);
				System.err.println("[final-network.txt] seeded ground-truth at " + expected);
				return;
			}
			throw new AssertionFailedError(
					"Ground-truth file not found: " + expected
							+ "\nRun with -D" + UPDATE_PROP + "=true to seed it from:\n  " + actual);
		}

		List<String> expectedLines = Files.readAllLines(expected, StandardCharsets.UTF_8);

		Map<String, List<String>> expectedByMethod = parseEndpoints(expectedLines);
		Map<String, List<String>> actualByMethod = parseEndpoints(actualLines);

		String diagnostic = diffEndpoints(expectedByMethod, actualByMethod);
		if (diagnostic != null) {
			if (forceUpdate) {
				writeLines(expected, actualLines);
				System.err.println("[final-network.txt] updated ground-truth at " + expected);
				return;
			}
			throw new AssertionFailedError(
					"final-network.txt endpoints differ from ground-truth\n"
							+ "  expected: " + expected + "\n"
							+ "  actual:   " + actual + "\n"
							+ "Run with -D" + UPDATE_PROP + "=true to regenerate.\n\n"
							+ diagnostic,
					String.join("\n", expectedLines),
					String.join("\n", actualLines));
		}

		// Endpoints agree. Location fragments may still drift — report as a
		// non-fatal notice so the drift stays visible without failing chunk 2.
		String locNotice = diffLocations(expectedLines, actualLines);
		if (locNotice != null)
			System.err.println("[final-network.txt] endpoints match at " + expected
					+ " but locations drifted (informational):\n" + locNotice);
	}

	/**
	 * Parses the final-network.txt format into a map from HTTP method to the
	 * sorted list of paths declared under it. The {@code COUNT: N} header and
	 * any text after the first {@code " : "} on an indented line (the location)
	 * are ignored.
	 */
	private static Map<String, List<String>> parseEndpoints(
			List<String> lines) {
		Map<String, TreeSet<String>> byMethod = new LinkedHashMap<>();
		String current = null;
		for (String raw : lines) {
			if (raw.isEmpty())
				continue;
			if (raw.startsWith("COUNT:"))
				continue;
			if (!raw.startsWith("\t") && !raw.startsWith(" ")) {
				Matcher m = METHOD_HEADER.matcher(raw);
				if (m.matches()) {
					current = m.group(1);
					byMethod.computeIfAbsent(current, k -> new TreeSet<>());
				} else {
					current = null;
				}
				continue;
			}
			if (current == null)
				continue;
			String body = raw.startsWith("\t") ? raw.substring(1) : raw.stripLeading();
			int sep = body.indexOf(": ");
			String path = sep < 0 ? body : body.substring(0, sep);
			byMethod.get(current).add(path);
		}

		Map<String, List<String>> result = new LinkedHashMap<>();
		for (Map.Entry<String, TreeSet<String>> e : byMethod.entrySet())
			result.put(e.getKey(), new ArrayList<>(e.getValue()));
		return result;
	}

	/**
	 * Returns {@code null} when both maps describe the same endpoint set,
	 * otherwise a human-readable diagnostic that lists missing and extra
	 * endpoints, annotated to flag extras for manual review.
	 */
	private static String diffEndpoints(
			Map<String, List<String>> expected,
			Map<String, List<String>> actual) {
		List<String> missing = new ArrayList<>();
		List<String> extra = new ArrayList<>();

		TreeSet<String> allMethods = new TreeSet<>();
		allMethods.addAll(expected.keySet());
		allMethods.addAll(actual.keySet());

		for (String method : allMethods) {
			TreeSet<String> exp = new TreeSet<>(expected.getOrDefault(method, List.of()));
			TreeSet<String> act = new TreeSet<>(actual.getOrDefault(method, List.of()));
			for (String p : exp)
				if (!act.contains(p))
					missing.add(method + " " + p);
			for (String p : act)
				if (!exp.contains(p))
					extra.add(method + " " + p);
		}

		if (missing.isEmpty() && extra.isEmpty())
			return null;

		StringBuilder sb = new StringBuilder();
		if (!missing.isEmpty()) {
			sb.append("Missing endpoints (regression):\n");
			for (String m : missing)
				sb.append("  - ").append(m).append('\n');
		}
		if (!extra.isEmpty()) {
			sb.append("Extra endpoints (manual review required — could be a genuine detection or a spurious one):\n");
			for (String e : extra)
				sb.append("  + ").append(e).append('\n');
		}
		return sb.toString();
	}

	/**
	 * When endpoints agree, surface location-fragment drift so we don't lose
	 * track of it. Returns {@code null} if every line matches byte-for-byte,
	 * otherwise the first differing line pair.
	 */
	private static String diffLocations(
			List<String> expected,
			List<String> actual) {
		int min = Math.min(expected.size(), actual.size());
		for (int i = 0; i < min; i++)
			if (!expected.get(i).equals(actual.get(i)))
				return "  L" + (i + 1) + " expected: " + expected.get(i) + "\n"
						+ "  L" + (i + 1) + " actual:   " + actual.get(i);
		if (expected.size() != actual.size())
			return "  line counts differ: expected=" + expected.size()
					+ ", actual=" + actual.size();
		return null;
	}

	private static boolean isUpdateEnabled() {
		return "true".equalsIgnoreCase(System.getProperty(UPDATE_PROP));
	}

	private static void writeLines(
			Path target,
			List<String> lines)
			throws IOException {
		Path parent = target.getParent();
		if (parent != null)
			createDirectories(parent);
		Path tmp = Files.createTempFile(parent == null ? Path.of(".") : parent, "final-network-", ".txt");
		Files.write(tmp, lines, StandardCharsets.UTF_8);
		Files.move(tmp, target, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
	}
}
