package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AsgTest extends AnalysisTestExecutor {

	@Test
	public void testAsg() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "asg";
		conf.programFile = "asg.py";
		perform(conf);
	}
}
