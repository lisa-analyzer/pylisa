package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BytesFormatTest extends AnalysisTestExecutor {

	@Test
	public void testBytesFormat() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "bytes-format";
		conf.programFile = "bytes-format.py";
		perform(conf);
	}
}
