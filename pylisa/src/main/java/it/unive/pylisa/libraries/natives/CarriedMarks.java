package it.unive.pylisa.libraries.natives;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * The marks that library models carry in the analysed state from one call to
 * the calls that follow it, given with the settings of the analysed program.
 * A model sets a carried mark with {@link ModelState#carry}; from then on,
 * every model call whose input state has the mark set on every execution
 * starts with the mark ({@link ModelState#assuming}), so the errors it raises
 * are kept apart as its own model's would be. A mark set on some executions
 * only is not carried: those of the other executions would be marked wrongly.
 * <p>
 * Each mark is a flag in the state, a global variable that the program does
 * not have, set to {@code false} at the program's entry so that joining an
 * execution that never set it with one that did never reads as set.
 * </p>
 *
 * @param names the names of the carried marks
 */
public record CarriedMarks(SortedSet<String> names) {

	/**
	 * No carried marks.
	 */
	public static final CarriedMarks NONE = new CarriedMarks(Set.of());

	/**
	 * Builds the carried marks.
	 *
	 * @param names the names of the carried marks
	 */
	public CarriedMarks(
			Set<String> names) {
		this(Collections.unmodifiableSortedSet(new TreeSet<>(names)));
	}

	/**
	 * Yields the flag of a carried mark in the analysed state.
	 *
	 * @param name     the name of the mark
	 * @param location the location of the statement reading or writing it
	 *
	 * @return the flag
	 */
	public static GlobalVariable flag(
			String name,
			CodeLocation location) {
		return new GlobalVariable(Untyped.INSTANCE, "$carried$" + name, location);
	}
}
