package it.unive.pylisa.microservices.fastapi;

import static it.unive.pylisa.microservices.MicroservicesTest.getLisaConf;

import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.io.IOException;

public class FastAPI {

	// @Test
	public void testminigcore() throws IOException {
		PyFrontend translator = new PyFrontend(
				"py-testcases/fastapi/miningcore/api.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getLisaConf("fastapi/miningcore/api");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}
}
