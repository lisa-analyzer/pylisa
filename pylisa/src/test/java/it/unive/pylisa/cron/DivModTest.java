package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DivModTest extends AnalysisTestExecutor {

	@Test
	public void testDivMod() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "divmod";
		conf.programFile = "divmod.py";
		perform(conf);
	}
}
