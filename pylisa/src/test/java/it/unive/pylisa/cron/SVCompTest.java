package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

/**
 * Tests on the SV-COMP python benchmarks: the benchmark {@code name} is
 * {@code py-testcases/svcomp/name/name.py}, and its expected results are in the
 * same folder.
 */
public class SVCompTest extends AnalysisTestExecutor {

	private void svcomp(
			String benchmark) {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "svcomp/" + benchmark;
		conf.programFile = "main.py";
		perform(conf);
	}

	@Test
	public void testArgs_assigned_call() {
		svcomp("args_assigned_call");
	}

	@Test
	public void testClasses_call() {
		svcomp("classes_call");
	}	
	
	@Test
	public void testAssignments_augmented() {
		svcomp("assignments_augmented");
	}
	
	@Test
	public void testAssignments_chained() {
		svcomp("assignments_chained");
	}

	@Test
	public void testArgs_default() {
		svcomp("args_default");
	}

	@Test
	public void testDirect_calls_return_call_unsafe() {
		svcomp("direct_calls_return_call_unsafe");
	}
}
