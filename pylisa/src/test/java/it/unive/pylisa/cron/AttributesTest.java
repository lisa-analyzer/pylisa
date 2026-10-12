package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AttributesTest extends AnalysisTestExecutor {

	@Test
	public void testAttributes() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "attributes";
		conf.programFile = "attributes.py";
		perform(conf);
	}
}
