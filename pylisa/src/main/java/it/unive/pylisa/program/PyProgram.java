package it.unive.pylisa.program;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import java.util.Objects;
import java.util.Optional;

/**
 * A translated Python program, together with the settings of the environment
 * it is analysed in. The settings travel with the program, so library models
 * read them from the call they model and no state outside the analysis is
 * needed.
 */
public class PyProgram extends Program {

	private final ProgramSettings settings;

	/**
	 * Builds an empty program.
	 *
	 * @param settings the settings of the environment the program is analysed
	 *                     in
	 */
	public PyProgram(
			ProgramSettings settings) {
		super(new PythonFeatures(), new PythonTypeSystem());
		this.settings = Objects.requireNonNull(settings);
	}

	/**
	 * Yields the settings of the environment the program is analysed in.
	 *
	 * @return the settings
	 */
	public ProgramSettings settings() {
		return settings;
	}

	/**
	 * Yields the setting of the given type of the program a statement belongs
	 * to.
	 *
	 * @param <T>       the type of the setting
	 * @param statement the statement
	 * @param type      the type the setting was registered under
	 *
	 * @return the setting, or empty if there is none of that type
	 *
	 * @throws IllegalStateException if the statement does not belong to a
	 *                                   program translated by pylisa
	 */
	public static <T> Optional<T> setting(
			Statement statement,
			Class<T> type) {
		Program program = statement.getProgram();
		if (!(program instanceof PyProgram py))
			throw new IllegalStateException(statement + " at " + statement.getLocation()
					+ " does not belong to a program translated by pylisa");
		return py.settings.get(type);
	}
}
