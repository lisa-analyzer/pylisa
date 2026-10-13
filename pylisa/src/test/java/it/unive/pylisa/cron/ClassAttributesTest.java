package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ClassAttributesTest extends AnalysisTestExecutor {

	@Test
	public void testClassAttributes() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "class-attributes";
		conf.programFile = "class-attributes.py";
		perform(conf);
	}
}
