package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AssignCallsTest extends AnalysisTestExecutor {

	@Test
	public void testAssignCalls() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "assign-calls";
		conf.programFile = "assign-calls.py";
		perform(conf);
	}
}
