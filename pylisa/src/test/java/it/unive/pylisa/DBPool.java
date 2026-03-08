package it.unive.pylisa;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;

import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.io.IOException;
import org.junit.Test;

public class DBPool {
	@Test
	public void testDBPool() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/mining-wave/database.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("database");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testConfig() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/mining-wave/config.py",
				false);
		Program program = translator.toLiSAProgram(false);
		LiSAConfiguration conf = getLisaConf("config");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}
}
