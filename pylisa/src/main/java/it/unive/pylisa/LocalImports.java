package it.unive.pylisa;

import java.util.Map;

/**
 * The names that the imports of a file bind to the local modules of the program
 * (the python files next to the analyzed one, see {@link LocalModules}).
 *
 * @param modules the names bound to modules (e.g. {@code m} for
 *                    {@code import m}, {@code a.b} for {@code import a.b},
 *                    {@code x} for {@code from m import x} when {@code m.x} is
 *                    a module), mapped to the names of their units
 * @param members the names bound to members of modules (e.g. {@code f} for
 *                    {@code from m import f}), mapped to the member
 */
public record LocalImports(
		Map<String, String> modules,
		Map<String, Member> members) {

	/**
	 * A member of a module.
	 *
	 * @param unit the name of the unit of the module
	 * @param name the name of the member in the module
	 */
	public record Member(
			String unit,
			String name) {
	}

	/**
	 * No imports of local modules.
	 */
	public static final LocalImports NONE = new LocalImports(Map.of(), Map.of());

	/**
	 * Yields the unit of the module bound to the given (possibly dotted) name.
	 *
	 * @param name the name, or {@code null}
	 *
	 * @return the name of the unit, or {@code null} if the name is not bound to
	 *             a local module
	 */
	public String module(
			String name) {
		return name == null ? null : modules.get(name);
	}

	/**
	 * Yields the member of a module bound to the given name.
	 *
	 * @param name the name
	 *
	 * @return the member, or {@code null} if the name is not bound to a member
	 *             of a local module
	 */
	public Member member(
			String name) {
		return members.get(name);
	}
}
