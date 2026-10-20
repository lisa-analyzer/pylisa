package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StarArgsTest extends AnalysisTestExecutor {

	@Test
	public void testStarArgs() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "star-args";
		conf.programFile = "star-args.py";
		perform(conf);
	}
}
