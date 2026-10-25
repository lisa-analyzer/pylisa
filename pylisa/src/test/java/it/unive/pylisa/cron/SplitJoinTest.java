package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SplitJoinTest extends AnalysisTestExecutor {

	@Test
	public void testSplitJoin() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "split-join";
		conf.programFile = "split-join.py";
		perform(conf);
	}
}
