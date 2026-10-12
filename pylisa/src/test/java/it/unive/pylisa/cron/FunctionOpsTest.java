package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class FunctionOpsTest extends AnalysisTestExecutor {

	@Test
	public void testFunctionOps() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "function-ops";
		conf.programFile = "function-ops.py";
		perform(conf);
	}
}
