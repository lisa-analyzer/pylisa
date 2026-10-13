package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AndOrTest extends AnalysisTestExecutor {

	@Test
	public void testAndOr() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "and-or";
		conf.programFile = "and-or.py";
		perform(conf);
	}
}
