package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BuiltinAttributesTest extends AnalysisTestExecutor {

	@Test
	public void testBuiltinAttributes() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "builtin-attributes";
		conf.programFile = "builtin-attributes.py";
		perform(conf);
	}
}
