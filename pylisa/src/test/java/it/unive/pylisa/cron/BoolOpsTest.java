package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BoolOpsTest extends AnalysisTestExecutor {

	@Test
	public void testBoolOps() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "bool-ops";
		conf.programFile = "bool-ops.py";
		perform(conf);
	}
}
