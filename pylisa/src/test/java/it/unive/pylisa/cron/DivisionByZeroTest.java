package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DivisionByZeroTest extends AnalysisTestExecutor {

	@Test
	public void testDivisionByZero() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "division-by-zero";
		conf.programFile = "division-by-zero.py";
		perform(conf);
	}
}
