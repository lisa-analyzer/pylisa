package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AttributeInitializationTest extends AnalysisTestExecutor {

	@Test
	public void testAttributeInitialization() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "attribute-initialization";
		conf.programFile = "attribute-initialization.py";
		perform(conf);
	}
}
