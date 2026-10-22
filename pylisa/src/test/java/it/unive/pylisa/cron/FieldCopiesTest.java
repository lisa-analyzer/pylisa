package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class FieldCopiesTest extends AnalysisTestExecutor {

	@Test
	public void testFieldCopies() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "field-copies";
		conf.programFile = "field-copies.py";
		perform(conf);
	}
}
