package it.unive.pylisa.program;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Facts about the environment a program is analysed in that its code does not
 * show, such as the values a deployment gives to parameters, which library
 * models read while the program is analysed. Each setting is registered under
 * the type library models look it up by, so a lookup never has to choose
 * between two candidates.
 * <p>
 * Instances are immutable; {@link #with} yields a new instance.
 * </p>
 */
public final class ProgramSettings {

	/**
	 * No settings: library models fall back to their defaults.
	 */
	public static final ProgramSettings NONE = new ProgramSettings(Map.of());

	private final Map<Class<?>, Object> settings;

	private ProgramSettings(
			Map<Class<?>, Object> settings) {
		this.settings = settings;
	}

	/**
	 * Yields these settings together with another one.
	 *
	 * @param <T>   the type the setting is looked up by
	 * @param type  the type the setting is looked up by
	 * @param value the setting
	 *
	 * @return the new settings
	 *
	 * @throws IllegalArgumentException if these settings already have one of
	 *                                      that type
	 */
	public <T> ProgramSettings with(
			Class<T> type,
			T value) {
		Objects.requireNonNull(type);
		Objects.requireNonNull(value);
		if (settings.containsKey(type))
			throw new IllegalArgumentException("Two settings of type " + type.getName());
		Map<Class<?>, Object> extended = new HashMap<>(settings);
		extended.put(type, value);
		return new ProgramSettings(Map.copyOf(extended));
	}

	/**
	 * Yields the setting of the given type.
	 *
	 * @param <T>  the type of the setting
	 * @param type the type the setting was registered under
	 *
	 * @return the setting, or empty if there is none of that type
	 */
	public <T> Optional<T> get(
			Class<T> type) {
		return Optional.ofNullable(settings.get(type)).map(type::cast);
	}

	@Override
	public String toString() {
		return settings.toString();
	}
}
