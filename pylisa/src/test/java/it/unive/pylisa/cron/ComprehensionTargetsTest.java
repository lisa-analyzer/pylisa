package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ComprehensionTargetsTest extends AnalysisTestExecutor {

	@Test
	public void testComprehensionTargets() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "comprehension-targets";
		conf.programFile = "comprehension-targets.py";
		perform(conf);
	}
}
