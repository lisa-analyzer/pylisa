package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MatchTest extends AnalysisTestExecutor {

	@Test
	public void testMatch() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "match";
		conf.programFile = "match.py";
		perform(conf);
	}
}
