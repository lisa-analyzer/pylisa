package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NoneOpsTest extends AnalysisTestExecutor {

	@Test
	public void testNoneOps() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "none-ops";
		conf.programFile = "none-ops.py";
		perform(conf);
	}
}
