package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MissingArgsTest extends AnalysisTestExecutor {

	@Test
	public void testMissingArgs() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "missing-args";
		conf.programFile = "missing-args.py";
		perform(conf);
	}
}
