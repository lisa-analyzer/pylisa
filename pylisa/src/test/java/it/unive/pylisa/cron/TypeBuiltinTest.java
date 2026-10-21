package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TypeBuiltinTest extends AnalysisTestExecutor {

	@Test
	public void testTypeBuiltin() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "type-builtin";
		conf.programFile = "type-builtin.py";
		perform(conf);
	}
}
