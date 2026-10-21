package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class LambdasTest extends AnalysisTestExecutor {

	@Test
	public void testLambdas() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "lambdas";
		conf.programFile = "lambdas.py";
		perform(conf);
	}
}
