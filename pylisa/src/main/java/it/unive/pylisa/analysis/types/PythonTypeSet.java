package it.unive.pylisa.analysis.types;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.type.TypeValue;
import it.unive.lisa.lattices.SetLattice;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.program.type.NoInfoType;
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

	public static final PythonTypeSet NO_INFO_TYPE = new PythonTypeSet(false, Set.of(NoInfoType.INSTANCE));
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

	/**
	 * Yields whether this set is below the given one. A set holding
	 * {@link NoInfoType} stands for every type, so every set is below it. The
	 * order must agree with this reading, since an environment reads an
	 * identifier it does not hold as {@link #NO_INFO_TYPE} (see
	 * {@link #unknownValue}): otherwise the least upper bound of two
	 * environments that name the same heap location differently, strongly in
	 * one and weakly in the other, would not be above both, and the fixpoint
	 * of the callers would never stabilise.
	 * <p>
	 * The order is a preorder: the sets holding {@link NoInfoType} are
	 * equivalent to each other and above every set but {@link #TOP}. They are
	 * not {@link #TOP} themselves, so {@link #isTop()} is false for them: a
	 * top set of types may be read as every type of the program, while the
	 * analysis dispatches a set holding {@link NoInfoType} on its other types
	 * and leaves the rest of the call open. Which of two
	 * equivalent sets a fixpoint keeps may depend on the order it visits the
	 * program in; both describe the same values.
	 * </p>
	 */
	@Override
	public boolean lessOrEqualAux(
			PythonTypeSet other)
			throws SemanticException {
		return other.elements.contains(NoInfoType.INSTANCE) || super.lessOrEqualAux(other);
	}

	/**
	 * Yields the greatest lower bound of this set and the given one. A set
	 * holding {@link NoInfoType} stands for every type, so the bound is the
	 * other set.
	 */
	@Override
	public PythonTypeSet glbAux(
			PythonTypeSet other)
			throws SemanticException {
		if (elements.contains(NoInfoType.INSTANCE))
			return other;
		if (other.elements.contains(NoInfoType.INSTANCE))
			return this;
		return super.glbAux(other);
	}

	/**
	 * Yields the types of an identifier the environment does not hold: any
	 * type. A missing identifier may be one that was never assigned, or a heap
	 * location that a join renamed (strong to weak), so nothing narrower is
	 * sound.
	 */
	@Override
	public PythonTypeSet unknownValue(
			Identifier id) {
		return NO_INFO_TYPE;
	}

	@Override
	public PythonTypeSet mk(
			Set<Type> set) {
		return new PythonTypeSet(true, set);
	}

}
