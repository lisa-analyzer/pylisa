package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ConversionsTest extends AnalysisTestExecutor {

	@Test
	public void testConversions() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "conversions";
		conf.programFile = "conversions.py";
		perform(conf);
	}
}
