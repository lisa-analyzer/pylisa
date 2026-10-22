package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NamespacesTest extends AnalysisTestExecutor {

	@Test
	public void testNamespaces() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "namespaces";
		conf.programFile = "namespaces.py";
		perform(conf);
	}
}
