package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class FunctionRefsTest extends AnalysisTestExecutor {

	@Test
	public void testFunctionRefs() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "function-refs";
		conf.programFile = "function-refs.py";
		perform(conf);
	}
}
