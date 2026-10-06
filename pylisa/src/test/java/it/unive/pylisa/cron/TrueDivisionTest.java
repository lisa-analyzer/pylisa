package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TrueDivisionTest extends AnalysisTestExecutor {

	@Test
	public void testTrueDivision() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "true-division";
		conf.programFile = "true-division.py";
		perform(conf);
	}
}
