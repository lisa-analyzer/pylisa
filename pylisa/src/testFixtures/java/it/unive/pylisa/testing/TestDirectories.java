package it.unive.pylisa.testing;

import java.nio.file.Path;

/**
 * The directories under {@code build/} where tests write files. Each test
 * task of a build writes under a directory of its own, and each process
 * running tests under one of its own within it, so that no process deletes or
 * reads what another one is writing, and one task does not remove what
 * another one left for inspection. The build removes the directories of a
 * task before the task starts.
 */
public final class TestDirectories {

	/**
	 * The system property that Gradle sets, in every process running tests,
	 * to an identifier of that process.
	 */
	private static final String WORKER_PROPERTY = "org.gradle.test.worker";

	/**
	 * The system property the build sets to the name of the test task.
	 */
	private static final String TASK_PROPERTY = "pylisa.test.task";

	private TestDirectories() {
	}

	/**
	 * Yields the directory of this process for the files of one kind.
	 *
	 * @param name the name of the kind of files, such as
	 *                 {@code analysis-results}
	 *
	 * @return {@code build/<name>/<task>/process-<process>}, relative to the
	 *             working directory of the tests
	 */
	public static Path of(
			String name) {
		return Path.of("build", name, System.getProperty(TASK_PROPERTY, "test"),
				"process-" + System.getProperty(WORKER_PROPERTY, "single"));
	}
}
