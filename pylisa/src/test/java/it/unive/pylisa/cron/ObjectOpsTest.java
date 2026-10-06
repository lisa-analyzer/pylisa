package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ObjectOpsTest extends AnalysisTestExecutor {

	@Test
	public void testObjectOps() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "object-ops";
		conf.programFile = "object-ops.py";
		perform(conf);
	}
}
