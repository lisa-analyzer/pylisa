package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ConstructorsTest extends AnalysisTestExecutor {

	@Test
	public void testConstructors() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "constructors";
		conf.programFile = "constructors.py";
		perform(conf);
	}
}
