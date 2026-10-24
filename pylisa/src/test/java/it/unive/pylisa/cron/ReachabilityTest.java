package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ReachabilityTest extends AnalysisTestExecutor {

	private void run(
			String dir) {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "reachability/" + dir;
		conf.programFile = "main.py";
		perform(conf);
	}

	@Test
	public void testAfterStructures() {
		run("after-structures");
	}

	@Test
	public void testAdjacentStructures() {
		run("adjacent-structures");
	}

	@Test
	public void testFunction() {
		run("function");
	}
}
