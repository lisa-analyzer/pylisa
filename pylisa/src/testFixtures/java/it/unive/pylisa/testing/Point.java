package it.unive.pylisa.testing;

import static org.junit.jupiter.api.Assertions.fail;

import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.HeapLocation;
import it.unive.pylisa.analysis.Val;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

/**
 * The analysis state right after one statement of the analysed program, joined
 * over every context in which that statement was analysed.
 * <p>
 * Program data is reached through dotted paths written as in Python: a
 * variable name followed by attribute names, such as {@code self.pub}. Reading
 * data at a point that no execution reaches normally is a test failure, since
 * nothing can be observed there; errors, on the other hand, can always be
 * inspected.
 * </p>
 */
public final class Point {

	private final StateView<?, ?> view;

	private final String description;

	private Point(
			StateView<?, ?> view,
			String description) {
		this.view = view;
		this.description = description;
	}

	/**
	 * Builds the point for an analysed state.
	 *
	 * @param view the state
	 *
	 * @return the point
	 */
	static Point of(
			StateView<?, ?> view) {
		return new Point(view, "after " + view.point() + " at " + view.point().getLocation());
	}

	/**
	 * Builds a point inside code that the analysis never reached.
	 *
	 * @param description where the point is
	 *
	 * @return the point
	 */
	static Point unanalysed(
			String description) {
		return new Point(null, description + " (never analysed)");
	}

	/**
	 * Yields whether some execution reaches this point normally.
	 *
	 * @return {@code true} if the normal execution state is not bottom
	 */
	public boolean isReachable() {
		return view != null && view.isReachable();
	}

	/**
	 * Yields the names of the types of the errors that may have been raised by
	 * the time this point is reached.
	 *
	 * @return the error type names
	 */
	public Set<String> errors() {
		return view == null ? Set.of() : view.errors();
	}

		/**
	 * Yields, for each error of a type raised on a line, the assumptions the
	 * branch that raised it depends on: empty for an error raised whatever
	 * the assumptions.
	 *
	 * @param type the qualified name of the exception type
	 * @param line the line of the statement that raised it
	 *
	 * @return the sets of assumptions, one per error
	 */
	public Set<Set<String>> errorMarks(
			String type,
			int line) {
		return view == null ? Set.of() : view.errorMarks(type, line);
	}

/**
	 * Yields the errors that may have been raised by the time this point is
	 * reached, each with the call that raised it.
	 *
	 * @return the errors
	 */
	public Set<ErrorSite> errorSites() {
		return view == null ? Set.of() : view.errorSites();
	}

	/**
	 * Yields whether every error that may have been raised by the time this
	 * point is reached was raised by a statement of the program, or by a call
	 * nested inside one: only such errors can be propagated to callers and
	 * attributed to a call site.
	 *
	 * @return {@code true} if every raising statement chains to a program
	 *             statement
	 */
	public boolean everyErrorRaisedWithinProgram() {
		if (view == null)
			return fail("no state " + description);
		return view.everyErrorRaisedWithinProgram();
	}

	/**
	 * Yields the value of a variable or of an attribute.
	 *
	 * @param path a variable name, optionally followed by attribute names
	 *                 separated by dots
	 *
	 * @return the value
	 */
	public Val value(
			String path) {
		return reachable().valueOf(expression(path)).orElseGet(() -> fail(path + " has no value " + description));
	}

	/**
	 * Yields the names of the runtime types a variable or an attribute may
	 * have.
	 *
	 * @param path a variable name, optionally followed by attribute names
	 *                 separated by dots
	 *
	 * @return the type names, sorted
	 */
	public Set<String> types(
			String path) {
		return reachable().typesOf(expression(path)).stream()
				.map(Object::toString)
				.collect(Collectors.toCollection(TreeSet::new));
	}

	/**
	 * Yields the single object a variable or an attribute points to.
	 *
	 * @param path a variable name, optionally followed by attribute names
	 *                 separated by dots
	 *
	 * @return the object
	 */
	public Obj object(
			String path) {
		return single(reachable(), expression(path), path);
	}

	/**
	 * Yields every object a variable or an attribute may point to.
	 *
	 * @param path a variable name, optionally followed by attribute names
	 *                 separated by dots
	 *
	 * @return the objects, possibly none
	 */
	public Set<Obj> objects(
			String path) {
		StateView<?, ?> view = reachable();
		SymbolicExpression reference = expression(path);
		// each object gets a reference of its own, so that reading one does
		// not read the others the path may point to
		return view.objectsPointedBy(reference).stream()
				.map(target -> new Obj(view, view.referenceTo(target), target))
				.collect(Collectors.toSet());
	}

	/**
	 * Yields the objects of the given type that exist at this point, whether
	 * or not a variable of the program points to them.
	 *
	 * @param typeName the name of the type, such as
	 *                     {@code mylib.Widget}
	 *
	 * @return the objects
	 */
	public Set<Obj> objectsOfType(
			String typeName) {
		StateView<?, ?> view = reachable();
		return view.objectsOfType(typeName).entrySet().stream()
				.map(entry -> new Obj(view, entry.getValue(), entry.getKey()))
				.collect(Collectors.toSet());
	}

	@Override
	public String toString() {
		return description;
	}

	/**
	 * Yields the single object a reference points to, failing if it may point
	 * to none or to several.
	 *
	 * @param view      the state
	 * @param reference the reference
	 * @param path      how the reference was reached, for failure messages
	 *
	 * @return the object
	 */
	static Obj single(
			StateView<?, ?> view,
			SymbolicExpression reference,
			String path) {
		Set<HeapLocation> targets = view.objectsPointedBy(reference);
		if (targets.size() != 1)
			return fail(path + " points to " + targets.size() + " objects " + targets + " at "
					+ view.point().getLocation());
		return new Obj(view, reference, targets.iterator().next());
	}

	private StateView<?, ?> reachable() {
		if (!isReachable())
			return fail("No execution reaches the point " + description);
		return view;
	}

	private SymbolicExpression expression(
			String path) {
		String[] names = path.split("\\.");
		SymbolicExpression expression = reachable().variable(names[0])
				.orElseGet(() -> fail("No variable " + names[0] + " " + description));
		for (int i = 1; i < names.length; i++)
			expression = StateView.field(expression, names[i]);
		return expression;
	}
}
