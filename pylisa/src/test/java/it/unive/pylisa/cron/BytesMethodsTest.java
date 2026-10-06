package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BytesMethodsTest extends AnalysisTestExecutor {

	@Test
	public void testBytesMethods() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "bytes-methods";
		conf.programFile = "bytes-methods.py";
		perform(conf);
	}
}
