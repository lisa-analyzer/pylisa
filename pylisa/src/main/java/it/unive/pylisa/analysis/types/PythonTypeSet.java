package it.unive.pylisa.analysis.types;

import it.unive.lisa.analysis.nonrelational.type.TypeValue;
import it.unive.lisa.lattices.SetLattice;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

public class PythonTypeSet
		extends
		SetLattice<PythonTypeSet, Type>
		implements
		TypeValue<PythonTypeSet> {

	/**
	 * The top element of the type lattice, representing all possible types.
	 */
	public static final PythonTypeSet TOP = new PythonTypeSet(true, Collections.emptySet());

	public static final PythonTypeSet NULL_TYPE = new PythonTypeSet(false, Set.of(NullType.INSTANCE));

	/**
	 * The bottom element of the type lattice, representing no types at all.
	 */
	public static final PythonTypeSet BOTTOM = new PythonTypeSet(false, Collections.emptySet());

	/**
	 * Builds the inferred types. The object built through this constructor
	 * represents an empty set of types.
	 */
	public PythonTypeSet() {
		this(true, Collections.emptySet());
	}

	/**
	 * Builds the inferred types, representing only the given {@link Type}.
	 *
	 * @param typeSystem the type system knowing about the types of the program
	 *                       where this element is created
	 * @param type       the type to be included in the set of inferred types
	 */
	public PythonTypeSet(
			TypeSystem typeSystem,
			Type type) {
		this(typeSystem, Collections.singleton(type));
	}

	/**
	 * Builds the inferred types, representing only the given set of
	 * {@link Type}s.
	 *
	 * @param typeSystem the type system knowing about the types of the program
	 *                       where this element is created
	 * @param types      the types to be included in the set of inferred types
	 */
	public PythonTypeSet(
			TypeSystem typeSystem,
			Set<Type> types) {
		this(true, typeSystem != null && types.equals(typeSystem.getTypes()) ? Collections.emptySet() : types);
	}

	/**
	 * Builds the inferred types, representing only the given set of
	 * {@link Type}s.
	 *
	 * @param isTop whether or not the set of types represents all possible
	 *                  types
	 * @param types the types to be included in the set of inferred types
	 */
	public PythonTypeSet(
			boolean isTop,
			Set<Type> types) {
		super(types, isTop);
	}

	@Override
	public Set<Type> getRuntimeTypes() {
		if (elements == null)
			Collections.emptySet();
		return elements;
	}

	@Override
	public PythonTypeSet top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return this == TOP || super.isTop();
	}

	@Override
	public PythonTypeSet bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return this == BOTTOM || super.isBottom();
	}

	@Override
	public PythonTypeSet unknownValue(
			Identifier id) {
		return NULL_TYPE;
	}

	@Override
	public PythonTypeSet mk(
			Set<Type> set) {
		return new PythonTypeSet(true, set);
	}

}
