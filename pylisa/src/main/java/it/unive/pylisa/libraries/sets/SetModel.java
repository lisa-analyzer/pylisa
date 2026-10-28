package it.unive.pylisa.libraries.sets;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 * The sets, that are objects in the heap whose elements (that have no order)
 * are their child {@value #ELEMENTS}: an element might be any of its values,
 * and the number of elements is not tracked (a set without the child has no
 * elements). The methods of sets that are not modeled might change their
 * elements in any way (see {@link #invalidate}).
 */
public final class SetModel {

	/**
	 * The child of the sets holding their elements. It is not a valid python
	 * identifier, so that it cannot be an attribute of the set.
	 */
	public static final String ELEMENTS = ListCreation.UNKNOWN_INDEX;

	private SetModel() {
	}

	/**
	 * Whether the values of the given type are sets.
	 *
	 * @param t the type
	 *
	 * @return whether the values are sets
	 */
	public static boolean isSet(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.SET);
	}

	/**
	 * Whether the sets do not define the given method (that is, it is not
	 * modeled).
	 *
	 * @param name the name of the method
	 *
	 * @return whether the method is not modeled
	 */
	public static boolean unmodeled(
			String name) {
		PyClassType set = PyClassType.lookup(LibrarySpecificationProvider.SET);
		if (set == null)
			return false;
		Set<Unit> units = new HashSet<>();
		units.add(set.getUnit());
		units.addAll(set.getUnit().getImmediateAncestors());
		for (Unit unit : units)
			if (!unit.getCodeMembersByName(name).isEmpty() || (unit instanceof CompilationUnit
					&& !((CompilationUnit) unit).getInstanceCodeMembersByName(name, true).isEmpty()))
				return false;
		return true;
	}

	/**
	 * Creates a new set, with the given elements.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param pp       the statement creating the set
	 * @param location the location of the set
	 * @param elements the states whose computed expressions are the elements,
	 *                     computed from {@code state} (none for an empty set)
	 * @param metas    where the receiver of the set is added, as it is a meta
	 *                     variable that the statement using the set must forget
	 *
	 * @return the state where the set has been created, whose computed
	 *             expression is the set
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Statement pp,
			CodeLocation location,
			Collection<AnalysisState<A>> elements,
			Collection<Identifier> metas)
			throws SemanticException {
		Type setType = PyClassType.lookup(LibrarySpecificationProvider.SET);
		ReferenceType reftype = new ReferenceType(setType);
		MemoryAllocation alloc = new MemoryAllocation(setType, location, false);
		InstrumentedReceiver set = new InstrumentedReceiver(reftype, false, location);
		AnalysisState<A> current = analysis.assign(Allocations.allocate(analysis, state, alloc, pp), set,
				new HeapReference(reftype, alloc, location), pp);
		HeapDereference deref = new HeapDereference(setType, set, location);
		// the heap tracks the fields accessed on each object, but until the
		// first access to any field in the program, the fields are unknown
		// for all objects (see PyNewObj): accessing __class__, that all
		// objects have, makes the elements of the new set known also when it
		// is empty
		AccessChild cls = new AccessChild(Untyped.INSTANCE, deref,
				new Variable(Untyped.INSTANCE, "__class__", location),
				location);
		current = analysis.smallStepSemantics(current, cls, pp).withExecutionExpressions(
				current.getExecutionExpressions());
		AccessChild child = elements(deref, location);
		boolean first = true;
		for (AnalysisState<A> element : elements)
			for (SymbolicExpression value : element.getExecutionExpressions()) {
				AnalysisState<A> added = analysis.assign(current, child, value, pp);
				// the first element is assigned, the others are joined with it
				current = first ? added : added.lub(current);
				first = false;
			}
		metas.add(set);
		return analysis.smallStepSemantics(current, set, pp);
	}

	/**
	 * Yields any of the elements of the set (e.g. when iterating it): the
	 * values of its child {@value #ELEMENTS}, none if it has no elements, and
	 * an unknown value if its elements are not known.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param set      the set
	 * @param pp       the program point reading the elements
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression set,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		// the child holding the elements, if the set has it (see
		// ListModel#shape, as the child has the name of the elements at
		// unknown indices of lists)
		ListModel.Shape shape = ListModel.shape(analysis, state, set, pp);
		if (shape.elements() == null)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		if (shape.elements().isEmpty())
			// no elements
			return state.bottomExecution();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.SET), set, loc);
		return analysis.smallStepSemantics(state, elements(deref, loc), pp);
	}

	/**
	 * Adds the given element to the set (a weak update of its elements).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param set      the set
	 * @param element  the element
	 * @param pp       the program point adding the element
	 *
	 * @return the state after the addition
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> add(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression set,
			SymbolicExpression element,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.SET), set, loc);
		return analysis.assign(state, elements(deref, loc), element, pp).lub(state);
	}

	/**
	 * Invalidates what is known about the elements of the set, as done by the
	 * methods of sets that are not modeled (e.g. {@code update}): any value
	 * might be one of its elements.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param set      the set
	 * @param pp       the program point modifying the set
	 *
	 * @return the state after the invalidation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> invalidate(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression set,
			ProgramPoint pp)
			throws SemanticException {
		return add(analysis, state, set, new PushAny(Untyped.INSTANCE, pp.getLocation()), pp);
	}

	private static AccessChild elements(
			HeapDereference set,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, set, new Variable(Untyped.INSTANCE, ELEMENTS, loc), loc);
	}
}
