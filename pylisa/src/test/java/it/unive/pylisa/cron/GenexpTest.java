package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class GenexpTest extends AnalysisTestExecutor {

	@Test
	public void testGenexp() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "genexp";
		conf.programFile = "genexp.py";
		perform(conf);
	}
}
