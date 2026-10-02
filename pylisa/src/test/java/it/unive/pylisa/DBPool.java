package it.unive.pylisa;

import static it.unive.pylisa.testutil.LiSAConfigs.getDefaultConf;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

public class DBPool {

	/**
	 * The folder of the programs, which the repository does not track.
	 */
	private static final String PROGRAMS = "py-testcases/mining-wave";

	@Test
	public void testDBPool() throws IOException {
		assumeTrue(Files.isDirectory(Path.of(PROGRAMS)), PROGRAMS + " is absent");
		PyFrontend translator = new PyFrontend(
				PROGRAMS + "/database.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getDefaultConf("database");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	@Test
	public void testConfig() throws IOException {
		assumeTrue(Files.isDirectory(Path.of(PROGRAMS)), PROGRAMS + " is absent");
		PyFrontend translator = new PyFrontend(
				PROGRAMS + "/config.py",
				false);
		Program program = translator.toLiSAProgram(true);
		LiSAConfiguration conf = getDefaultConf("config");
		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}
}
