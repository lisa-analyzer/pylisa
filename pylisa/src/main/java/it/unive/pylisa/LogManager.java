package it.unive.pylisa;

import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.core.config.Configurator;

public class LogManager {
	private static final String LISA_LOGGER = "it.unive.lisa";
	private static final String PYLISA_LOGGER = "it.unive.pylisa";

	public static void setLogLevel(
			Level level) {
		Configurator.setLevel(LISA_LOGGER, level);
		Configurator.setLevel(PYLISA_LOGGER, level);
	}
}
