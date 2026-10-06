package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BytesTest extends AnalysisTestExecutor {

	@Test
	public void testBytes() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "bytes";
		conf.programFile = "bytes.py";
		perform(conf);
	}
}
