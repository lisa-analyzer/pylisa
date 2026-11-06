package it.unive.pylisa;

import it.unive.lisa.logging.Log4jConfig;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.core.config.Configurator;

public class LogManager {
	private static final String LISA_LOGGER = "it.unive.lisa";
	private static final String PYLISA_LOGGER = "it.unive.pylisa";

	public static void setLogLevel(
			Level level) {
		// without a log4j configuration, lisa installs its default one when
		// LiSA or LiSAConfiguration are first accessed, replacing the one where
		// the levels are set: it is installed beforehand to keep the levels
		if (!Log4jConfig.isLog4jConfigured())
			Log4jConfig.initializeLogging();
		Configurator.setLevel(LISA_LOGGER, level);
		Configurator.setLevel(PYLISA_LOGGER, level);
	}
}
