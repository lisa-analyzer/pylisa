package it.unive.pylisa.cli;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.testing.TestDirectories;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import org.junit.jupiter.api.Test;

/**
 * Checks pylisa's command line: it analyses a Python program with pylisa's
 * analysis and generic checks, reports what they find, and refuses a wrong
 * command line.
 */
class PyLisaMainTest {

	private static final String PROGRAMS = "src/test/resources/programs/checks/always_raises/";

	@Test
	void aStatementThatAlwaysRaisesIsReportedAndFailsTheRun() throws Exception {
		Output output = run(PROGRAMS + "wrong_binding.py");
		assertEquals(1, output.status, output.err);
		assertTrue(output.out.contains("Always raises: 1"), output.out);
		assertTrue(output.out.contains("every execution that completes raises an exception, such as [builtins.TypeError]"), output.out);
		assertTrue(output.out.contains("CP_BSS/best-effort"), output.out);
	}

	@Test
	void aProgramWithoutFindingsSucceeds() throws Exception {
		Output output = run(PROGRAMS + "right_binding.py");
		assertEquals(0, output.status, output.out);
		assertTrue(output.out.contains("Always raises: none"), output.out);
	}

	@Test
	void theConfigurationCanBeChosen() throws Exception {
		Output output = run("--config", "cp", PROGRAMS + "right_binding.py");
		assertTrue(output.out.contains("CP/best-effort"), output.out);
	}

	@Test
	void aWrongCommandLineIsRefused() throws Exception {
		assertEquals(2, run().status);
		assertEquals(2, run("--config", "nope", PROGRAMS + "right_binding.py").status);
		Output missing = run("no/such/file.py");
		assertEquals(2, missing.status);
		assertTrue(missing.err.contains("no such file"), missing.err);
	}

	private record Output(int status, String out, String err) {
	}

	private static Output run(
			String... args)
			throws Exception {
		String[] all = new String[args.length + 2];
		all[0] = "--workdir";
		all[1] = TestDirectories.of("cli").toString();
		System.arraycopy(args, 0, all, 2, args.length);
		ByteArrayOutputStream out = new ByteArrayOutputStream();
		ByteArrayOutputStream err = new ByteArrayOutputStream();
		int status = PyLisaMain.run(all, new PrintStream(out, true, StandardCharsets.UTF_8),
				new PrintStream(err, true, StandardCharsets.UTF_8));
		return new Output(status, out.toString(StandardCharsets.UTF_8), err.toString(StandardCharsets.UTF_8));
	}
}
