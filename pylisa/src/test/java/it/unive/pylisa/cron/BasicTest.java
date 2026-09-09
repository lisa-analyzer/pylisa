package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BasicTest extends AnalysisTestExecutor {

	@Test
	public void testBasic() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "basic";
		conf.programFile = "basic.py";
		perform(conf);
	}
}
