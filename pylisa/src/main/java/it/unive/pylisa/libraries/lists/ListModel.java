package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.HashSet;
import java.util.Set;
import java.util.TreeSet;

/**
 * The model of lists (see {@link ListCreation}): the element at index {@code i}
 * of a list is its child {@code i} (an integer constant), and the elements at
 * unknown indices are its child {@value ListCreation#UNKNOWN_INDEX}. The length
 * of a list is not stored, as it changes when the list is modified: it is
 * computed from the elements, and it is known when all of them are at known
 * indices (that is, the list has no elements at unknown indices).
 */
public final class ListModel {

	private ListModel() {
	}

	/**
	 * Whether the values of the given type are lists.
	 *
	 * @param t the type
	 *
	 * @return whether the values are lists
	 */
	public static boolean isList(
			Type t) {
		return ListOperations.LIST.test(t);
	}

	/**
	 * Whether the lists do not define the given method (that is, it is not
	 * modeled).
	 *
	 * @param name the name of the method
	 *
	 * @return whether the method is not modeled
	 */
	public static boolean unmodeled(
			String name) {
		PyClassType list = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		if (list == null)
			return false;
		Set<Unit> units = new HashSet<>();
		units.add(list.getUnit());
		units.addAll(list.getUnit().getImmediateAncestors());
		for (Unit unit : units)
			if (!unit.getCodeMembersByName(name).isEmpty() || (unit instanceof CompilationUnit
					&& !((CompilationUnit) unit).getInstanceCodeMembersByName(name, true).isEmpty()))
				return false;
		return true;
	}

	// the objects the list points to, or null if they are not known
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<AllocationSite> sites(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || heap.isTop() || heap.isBottom())
			return null;
		Set<AllocationSite> sites = new HashSet<>();
		for (SymbolicExpression rec : analysis.rewrite(state, list, pp))
			if (rec instanceof MemoryPointer && ((MemoryPointer) rec).getReferencedLocation() instanceof AllocationSite)
				sites.add((AllocationSite) ((MemoryPointer) rec).getReferencedLocation());
			else if (rec instanceof Identifier && heap.knowsIdentifier((Identifier) rec)) {
				AllocationSites pointed = heap.getState((Identifier) rec);
				if (pointed.isTop() || pointed.isBottom())
					return null;
				sites.addAll(pointed.elements());
			} else
				return null;
		return sites.isEmpty() ? null : sites;
	}

	/**
	 * The length of the list, that is known if the list is a single object (and
	 * not the summary of many, e.g. the lists created by a literal in a loop)
	 * built by a list literal, whose elements are all at known indices.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point using the length
	 *
	 * @return the length, or {@code null} if it is not known
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Integer length(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		Set<AllocationSite> sites = sites(analysis, state, list, pp);
		if (sites == null)
			return null;
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap.fields.isTop() || heap.fields.function == null)
			return null;
		Integer length = null;
		for (AllocationSite site : sites) {
			if (site.isWeak())
				// the summary of many lists, that might have different
				// lengths
				return null;
			ExpressionSet fields = heap.fields.getState(site);
			if (fields.isTop())
				return null;
			Set<Integer> indexes = new TreeSet<>();
			for (SymbolicExpression field : fields)
				if (field instanceof Variable && ((Variable) field).getName().equals(ListCreation.UNKNOWN_INDEX))
					// elements at unknown indices
					return null;
				else if (field instanceof Constant && ((Constant) field).getValue() instanceof Integer)
					indexes.add((Integer) ((Constant) field).getValue());
			int n = indexes.size();
			if (!indexes.isEmpty() && (indexes.iterator().next() != 0 || ((TreeSet<Integer>) indexes).last() != n - 1))
				// e.g. an element assigned past the end
				return null;
			if (length != null && length != n)
				return null;
			length = n;
		}
		return length;
	}

	/**
	 * Invalidates what is known about the elements of the list, as done by the
	 * operations on lists that are not modeled (e.g. {@code pop}): the elements
	 * at known indices become unknown, and the list has elements at unknown
	 * indices (so that its length is unknown).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point modifying the list
	 *
	 * @return the state after the invalidation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> invalidate(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference deref = new HeapDereference(listType, list, loc);
		AnalysisState<A> result = state;
		Set<AllocationSite> sites = sites(analysis, state, list, pp);
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (sites != null && !heap.fields.isTop() && heap.fields.function != null)
			for (AllocationSite site : sites)
				for (SymbolicExpression field : heap.fields.getState(site))
					if (field instanceof Constant)
						result = analysis.assign(result, new AccessChild(Untyped.INSTANCE, deref, field, loc),
								new PushAny(Untyped.INSTANCE, loc), pp);
		AccessChild unknown = new AccessChild(Untyped.INSTANCE, deref,
				new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
		return analysis.assign(result, unknown, new PushAny(Untyped.INSTANCE, loc), pp);
	}
}
