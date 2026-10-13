package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class LenTest extends AnalysisTestExecutor {

	@Test
	public void testLen() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "len";
		conf.programFile = "len.py";
		perform(conf);
	}
}
