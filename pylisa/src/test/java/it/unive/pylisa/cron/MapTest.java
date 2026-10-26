package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MapTest extends AnalysisTestExecutor {

	@Test
	public void testMap() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "map";
		conf.programFile = "map.py";
		perform(conf);
	}
}
