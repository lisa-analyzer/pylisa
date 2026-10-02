package it.unive.pylisa.testing;

import static org.junit.jupiter.api.Assertions.fail;

import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.HeapLocation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

/**
 * One heap object at a program point, reached through a reference expression.
 * Two instances are equal when they denote the same abstract object, that is,
 * the same allocation site.
 */
public final class Obj {

	private final StateView<?, ?> view;

	private final SymbolicExpression reference;

	private final HeapLocation location;

	/**
	 * Builds the object.
	 *
	 * @param view      the state the object lives in
	 * @param reference an expression that points to the object
	 * @param location  the heap location of the object
	 */
	Obj(
			StateView<?, ?> view,
			SymbolicExpression reference,
			HeapLocation location) {
		this.view = view;
		this.reference = reference;
		this.location = location;
	}

	/**
	 * Yields the name of the abstract location of this object, which
	 * identifies the allocation site that created it.
	 *
	 * @return the location name
	 */
	public String location() {
		return location.getName();
	}

	/**
	 * Yields the name of the type of this object.
	 *
	 * @return the type name
	 */
	public String type() {
		Set<Type> types = view.typesOf(new HeapDereference(Untyped.INSTANCE, reference, PySyntheticLocation.INSTANCE));
		if (types.size() != 1)
			return fail("Object " + this + " has " + types.size() + " possible types: " + types);
		return types.iterator().next().toString();
	}

	/**
	 * Yields the value of a field of this object.
	 *
	 * @param name the field name
	 *
	 * @return the value
	 */
	public Val field(
			String name) {
		return view.valueOf(StateView.field(reference, name))
				.orElseGet(() -> fail("Field " + name + " of " + this + " has no value"));
	}

	/**
	 * Yields the value of a field of this object, if the field has one here.
	 *
	 * @param name the field name
	 *
	 * @return the value, or empty if the field is not set at this point
	 */
	public Optional<Val> fieldIfSet(
			String name) {
		SymbolicExpression field = StateView.field(reference, name);
		return view.isStored(field) ? view.valueOf(field) : Optional.empty();
	}

	/**
	 * Yields the names of the runtime types the value of a field may have. For
	 * a field holding a function, such as a registered callback, the type names
	 * the function: its qualified name, such as {@code __main__.Listener@4:0.cb}.
	 *
	 * @param name the field name
	 *
	 * @return the type names, sorted
	 */
	public Set<String> fieldTypes(
			String name) {
		return view.typesOf(StateView.field(reference, name)).stream()
				.map(Object::toString)
				.collect(Collectors.toCollection(TreeSet::new));
	}

	/**
	 * Yields the object a reference field of this object points to.
	 *
	 * @param name the field name
	 *
	 * @return the pointed object
	 */
	public Obj ref(
			String name) {
		return Point.single(view, StateView.field(reference, name), this + "." + name);
	}

	/**
	 * Yields the objects a reference field of this object may point to.
	 *
	 * @param name the field name
	 *
	 * @return the pointed objects, possibly none
	 */
	public Set<Obj> refs(
			String name) {
		SymbolicExpression field = StateView.field(reference, name);
		return view.objectsPointedBy(field).stream()
				.map(target -> new Obj(view, field, target))
				.collect(Collectors.toSet());
	}

	@Override
	public boolean equals(
			Object other) {
		return other instanceof Obj obj && location().equals(obj.location());
	}

	@Override
	public int hashCode() {
		return location().hashCode();
	}

	@Override
	public String toString() {
		return reference + " (" + location() + ")";
	}
}
