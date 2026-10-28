package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
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
				if (field instanceof Variable && (((Variable) field).getName().equals(ListCreation.UNKNOWN_INDEX)
						|| ((Variable) field).getName().equals(SHRUNK)))
					// elements at unknown indices, or removed
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
	 * operations on lists that are not modeled (e.g. {@code remove}): the
	 * elements at known indices become unknown, the list has elements at
	 * unknown indices, and elements might have been removed (see
	 * {@link #SHRUNK}).
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
		return shrink(analysis, analysis.assign(result, unknown, new PushAny(Untyped.INSTANCE, loc), pp), list, pp);
	}

	/**
	 * The child of the lists from which elements might have been removed (e.g.
	 * by {@code pop}): the elements at their known indices might be at other
	 * indices, or might have been removed, so that the lists are not known to
	 * have any element (see {@link #shape}), and their length is not known (see
	 * {@link #length}). The values of their elements are still the ones of the
	 * elements at known and unknown indices.
	 */
	public static final String SHRUNK = "$shrunk";

	/**
	 * Records that elements might have been removed from the list (see
	 * {@link #SHRUNK}).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point removing the elements
	 *
	 * @return the state where the list is marked
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> shrink(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference deref = new HeapDereference(listType, list, loc);
		AccessChild shrunk = new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, SHRUNK, loc),
				loc);
		AnalysisState<A> result = analysis.assign(state, shrunk, new Constant(BoolType.INSTANCE, true, loc), pp);
		// the list might have lost its last element
		return setNonEmpty(analysis, result, list, false, pp).withExecutionExpressions(state.getExecutionExpressions());
	}

	/**
	 * The child of the lists that is {@code True} if the list surely has an
	 * element, and {@code False} if it is not known (e.g. after {@code if xs:},
	 * {@code xs} has an element, see {@code PyDomain}). It is {@code False}
	 * when the list is created (see {@link ListCreation}), so that it is
	 * defined for all the lists, and it is set to {@code False} whenever
	 * elements might be removed (see {@link #shrink}). It complements the
	 * elements that the list surely has at known indices (see {@link #shape}),
	 * as it does not tell at which indices the elements are.
	 */
	public static final String NONEMPTY = "$nonempty";

	/**
	 * Records whether the list surely has an element (see {@link #NONEMPTY}).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param nonEmpty whether the list surely has an element
	 * @param pp       the program point
	 *
	 * @return the updated state, whose computed expressions are the ones of
	 *             {@code state}
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> setNonEmpty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			boolean nonEmpty,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		AccessChild child = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
				new Variable(Untyped.INSTANCE, NONEMPTY, loc), loc);
		return analysis.assign(state, child, new Constant(BoolType.INSTANCE, nonEmpty, loc), pp)
				.withExecutionExpressions(state.getExecutionExpressions());
	}

	/**
	 * Yields whether the list surely has an element according to
	 * {@link #NONEMPTY}: this holds only for lists that are single objects (and
	 * not the summary of many, as the lists created by a literal in a loop),
	 * whose child is surely {@code True}.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point using the list
	 *
	 * @return whether the list surely has an element
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean nonEmpty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		Set<AllocationSite> sites = sites(analysis, state, list, pp);
		if (sites == null || sites.stream().anyMatch(AllocationSite::isWeak))
			return false;
		CodeLocation loc = pp.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		AccessChild child = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
				new Variable(Untyped.INSTANCE, NONEMPTY, loc), loc);
		return analysis.satisfies(state, child, pp) == Satisfiability.SATISFIED;
	}

	// whether the value is surely the given int
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean surely(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			int c,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, pp);
		return !types.isEmpty() && types.stream().allMatch(ListOperations.INT)
				&& analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, value,
						new Constant(Int32Type.INSTANCE, c, loc), ComparisonEq.INSTANCE, loc),
						pp) == Satisfiability.SATISFIED;
	}

	/**
	 * Yields the elements of a list or tuple of known length, in order, when
	 * each of them has a single value.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param iterable the list or tuple
	 * @param pp       the program point using the elements
	 *
	 * @return the elements, or {@code null} if they are not known
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<SymbolicExpression> exactElements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterable,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, iterable, pp);
		if (types.size() != 1 || !(isList(types.iterator().next()) || Unpacking.isTuple(types.iterator().next())))
			return null;
		Shape shape = shape(analysis, state, iterable, pp);
		if (shape.length() == null)
			return null;
		HeapDereference deref = new HeapDereference(types.iterator().next().asPointerType().getInnerType(), iterable,
				loc);
		List<SymbolicExpression> elements = new ArrayList<>();
		for (int i = 0; i < shape.length(); i++) {
			ExpressionSet element = analysis.smallStepSemantics(state,
					new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, i, loc), loc), pp)
					.getExecutionExpressions();
			if (element.elements().size() != 1)
				return null;
			elements.add(element.elements().iterator().next());
		}
		return elements;
	}

	/**
	 * What is known about the elements of a list.
	 *
	 * @param length   the length of the list, or {@code null} if it is not
	 *                     known
	 * @param prefix   the number of elements at known indices that the list
	 *                     surely has (that is, the list has at least the
	 *                     elements at indices {@code 0, ..., prefix - 1})
	 * @param elements the elements of the list (the children of the objects it
	 *                     points to), or {@code null} if they are not known
	 */
	public record Shape(
			Integer length,
			int prefix,
			Set<SymbolicExpression> elements) {
	}

	/**
	 * Yields what is known about the elements of the list.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point using the list
	 *
	 * @return the shape of the list
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Shape shape(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		Set<AllocationSite> sites = sites(analysis, state, list, pp);
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (sites == null || heap.fields.isTop() || heap.fields.function == null)
			return new Shape(null, 0, null);
		Integer prefix = null;
		Set<SymbolicExpression> elements = new HashSet<>();
		for (AllocationSite site : sites) {
			ExpressionSet fields = heap.fields.getState(site);
			if (fields.isTop())
				return new Shape(null, 0, null);
			Set<Integer> indexes = new HashSet<>();
			boolean shrunk = false;
			for (SymbolicExpression field : fields)
				if (field instanceof Variable && ((Variable) field).getName().equals(SHRUNK))
					shrunk = true;
				else if (field instanceof Variable
						&& ((Variable) field).getName().equals(ListCreation.UNKNOWN_INDEX))
					elements.add(field);
				else if (field instanceof Constant && ((Constant) field).getValue() instanceof Integer) {
					indexes.add((Integer) ((Constant) field).getValue());
					elements.add(field);
				}
			int k = 0;
			while (indexes.contains(k))
				k++;
			// the summary of many lists, whose elements at known indices might
			// be the ones of some of them only
			// (and, after elements are removed, the elements at known indices
			// might be at other indices, or removed)
			int surely = site.isWeak() || shrunk ? 0 : k;
			prefix = prefix == null ? surely : Math.min(prefix, surely);
		}
		return new Shape(length(analysis, state, list, pp), prefix == null ? 0 : prefix, elements);
	}

	/**
	 * Reads the element at the given index of the list ({@code list[index]}),
	 * raising {@code IndexError} if the index might be out of range, and
	 * {@code TypeError} if it might not be an {@code int}. A negative index
	 * counts from the end of the list. The element is read exactly if the index
	 * is surely equal to the index of an element that the list surely has,
	 * otherwise the result is any of the elements of the list.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the list and the index have been
	 *                     evaluated
	 * @param list     the list
	 * @param index    the index
	 * @param thrower  the statement reading the element
	 * @param raising  where the exceptions raised by the read are collected
	 *
	 * @return the state after the read, where the executions raising exceptions
	 *             have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> read(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression index,
			Statement thrower,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference deref = new HeapDereference(listType, list, loc);
		Shape shape = shape(analysis, state, list, thrower);

		AnalysisState<A> result = state.bottomExecution();
		boolean typeError = false, indexError = false, any = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, index, thrower);
		if (types.isEmpty() || types.stream().anyMatch(t -> !ListOperations.INT.test(t)))
			// e.g. a str, or an unknown value
			typeError = true;
		// an unknown value (e.g. returned by an unknown function) might be an
		// int
		if (types.isEmpty() || types.stream().anyMatch(t -> ListOperations.INT.test(t) || t.isUntyped())) {
			// the indices the index might be equal to (also when it is not a
			// constant, e.g. -1 or a variable): the ones of the elements
			// that the list surely has
			int from = shape.length() == null ? 0 : -shape.length();
			int to = shape.length() == null ? shape.prefix() : shape.length();
			Integer exact = null;
			for (int c = from; c < to && exact == null; c++)
				if (analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, index,
						new Constant(Int32Type.INSTANCE, c, loc), ComparisonEq.INSTANCE, loc),
						thrower) == Satisfiability.SATISFIED)
					exact = c;
			if (exact != null)
				result = result.lub(element(analysis, state, deref,
						exact < 0 ? shape.length() + exact : exact, thrower));
			else if (shape.length() != null) {
				// whether the index is in range
				Constant low = new Constant(Int32Type.INSTANCE, -shape.length(), loc);
				Constant high = new Constant(Int32Type.INSTANCE, shape.length(), loc);
				Satisfiability in = analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE,
						new BinaryExpression(BoolType.INSTANCE, index, low, ComparisonGe.INSTANCE, loc),
						new BinaryExpression(BoolType.INSTANCE, index, high, ComparisonLt.INSTANCE, loc),
						LogicalAnd.INSTANCE, loc), thrower);
				if (in != Satisfiability.NOT_SATISFIED)
					any = true;
				if (in != Satisfiability.SATISFIED)
					indexError = true;
			} else {
				any = true;
				// 0 and -1 are in range if the list surely has an element,
				// that is any of its elements if its length is not known
				indexError = !((shape.prefix() > 0 || nonEmpty(analysis, state, list, thrower))
						&& (surely(analysis, state, index, 0, thrower) || surely(analysis, state, index, -1, thrower)));
			}
		}

		if (any)
			if (shape.elements() == null)
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), thrower));
			else
				for (SymbolicExpression field : shape.elements())
					result = result.lub(analysis.smallStepSemantics(state,
							new AccessChild(Untyped.INSTANCE, deref, field, loc), thrower));
		if (indexError)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.INDEX_ERROR);
		if (typeError)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.TYPE_ERROR);
		return result;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> element(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			int index,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		return analysis.smallStepSemantics(state,
				new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, index, loc), loc), pp);
	}

	/**
	 * Whether the values of the given type are slices (e.g. {@code 1:3} in
	 * {@code lst[1:3]}).
	 *
	 * @param t the type
	 *
	 * @return whether the values are slices
	 */
	public static boolean isSlice(
			Type t) {
		return t.isUnitType() && t.asUnitType().getUnit().getName().equals(LibrarySpecificationProvider.SLICE);
	}

	/**
	 * Reads the slice of the list ({@code list[start:stop:step]}): the result
	 * is a new list, whose elements are some of the elements of the list, at
	 * unknown indices. A slice never raises {@code IndexError}, while
	 * {@code TypeError} is raised if its bounds might not be {@code int}s (or
	 * {@code None}), and {@code ValueError} if its step might be {@code 0}.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the list and the slice have been
	 *                            evaluated
	 * @param list            the list
	 * @param slice           the slice
	 * @param owner           the expression the new list belongs to
	 * @param thrower         the statement reading the slice
	 * @param raising         where the exceptions raised by the read are
	 *                            collected
	 *
	 * @return the state after the read, where the executions raising exceptions
	 *             have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> slice(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression slice,
			Expression owner,
			Statement thrower,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = owner.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference source = new HeapDereference(listType, list, loc);
		Shape shape = shape(analysis, state, list, owner);

		boolean typeError = !boundsTyped(analysis, state, slice, owner);
		Satisfiability zero = stepZero(analysis, state, slice, owner);

		// the slice is allocated at a location different from the one of the
		// subscript (see DerivedLocation)
		ListCreation creation = new ListCreation(owner.getCFG(), new DerivedLocation(loc, "slice"));
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				new StatementStore<>(state));
		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression copy : created.getExecutionExpressions()) {
			AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, copy, loc),
					new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
			AnalysisState<A> filled = created.bottomExecution();
			if (shape.elements() == null)
				filled = analysis.assign(created, unknown, new PushAny(Untyped.INSTANCE, loc), owner);
			else
				for (SymbolicExpression field : shape.elements())
					for (SymbolicExpression element : analysis.smallStepSemantics(created,
							new AccessChild(Untyped.INSTANCE, source, field, loc), owner).getExecutionExpressions())
						filled = filled.lub(analysis.assign(created, unknown, element, owner));
			if (filled.getExecutionState().isBottom())
				// the list is empty: so is the slice
				filled = created;
			result = result.lub(filled.withExecutionExpression(copy));
		}
		owner.getMetaVariables().addAll(creation.getMetaVariables());
		if (zero == Satisfiability.SATISFIED)
			result = result.bottomExecution();
		if (zero != Satisfiability.NOT_SATISFIED)
			// a step of 0
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.VALUE_ERROR));
		if (typeError)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.TYPE_ERROR);
		return result;
	}

	/**
	 * Assigns the elements of {@code value} to the slice of the list
	 * ({@code list[start:stop:step] = value}): they replace the elements of the
	 * slice, so that the list might have more or less elements, at other
	 * indices. The list is then one whose elements might have been removed (see
	 * {@link #SHRUNK}), whose elements at unknown indices are joined with the
	 * elements of {@code value} (see {@link Iteration#elements}).
	 * {@code TypeError} is raised if the bounds might not be {@code int}s (or
	 * {@code None}), or if {@code value} might not be iterable (see
	 * {@link Iteration#iterable}); {@code ValueError} is raised if the step
	 * might be {@code 0}, and might be raised if it might be different from
	 * {@code 1} (the elements of an extended slice are replaced only by as many
	 * elements).
	 *
	 * @param analysis the analysis
	 * @param state    the state where the list, the slice and the value have
	 *                     been evaluated
	 * @param list     the list
	 * @param slice    the slice
	 * @param value    the value
	 * @param thrower  the statement assigning the slice
	 * @param metas    where the meta variables of the elements of the value are
	 *                     added (e.g. the tuples of {@code d.items()})
	 * @param raising  where the exceptions raised by the assignment are
	 *                     collected
	 *
	 * @return the state after the assignment, where the executions raising
	 *             exceptions have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> assignSlice(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression slice,
			SymbolicExpression value,
			Statement thrower,
			Collection<Identifier> metas,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		boolean typed = boundsTyped(analysis, state, slice, thrower);
		Satisfiability iterable = Iteration.iterable(analysis, state, value, thrower);
		Satisfiability zero = stepZero(analysis, state, slice, thrower);
		// whether the step might be different from 1 (and from None)
		boolean extended = false;
		if (slice instanceof TernaryExpression) {
			SymbolicExpression step = ((TernaryExpression) slice).getRight();
			Set<Type> types = analysis.getRuntimeTypesOf(state, step, thrower);
			extended = !types.stream().allMatch(Type::isNullType)
					&& analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step,
							new Constant(Int32Type.INSTANCE, 1, loc), ComparisonEq.INSTANCE, loc),
							thrower) != Satisfiability.SATISFIED;
		}

		AnalysisState<A> result = state.bottomExecution();
		if (typed && iterable != Satisfiability.NOT_SATISFIED && zero != Satisfiability.SATISFIED) {
			Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
			AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
					new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
			// the value might have no element (e.g. xs[0:2] = [])
			AnalysisState<A> assigned = shrink(analysis, state, list, thrower);
			AnalysisState<A> elements = Iteration.elements(analysis, state, null, value, thrower, metas, raising);
			if (!elements.getExecutionState().isBottom()) {
				AnalysisState<A> shrunk = shrink(analysis, elements, list, thrower);
				// weak update: the new elements are joined with the previous
				// ones, that might still be elements of the list
				for (SymbolicExpression element : elements.getExecutionExpressions())
					assigned = assigned.lub(analysis.assign(shrunk, unknown, element, thrower).lub(shrunk));
			}
			result = assigned;
			if (extended)
				// an extended slice, whose length might not be the one of the
				// value
				raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.VALUE_ERROR);
		}
		if (zero != Satisfiability.NOT_SATISFIED)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.VALUE_ERROR);
		if (!typed || iterable != Satisfiability.SATISFIED)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.TYPE_ERROR);
		return result;
	}

	// whether the bounds of the slice are surely ints or None
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean boundsTyped(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression slice,
			ProgramPoint pp)
			throws SemanticException {
		if (!(slice instanceof TernaryExpression))
			return false;
		TernaryExpression bounds = (TernaryExpression) slice;
		for (SymbolicExpression bound : new SymbolicExpression[] { bounds.getLeft(), bounds.getMiddle(),
				bounds.getRight() }) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, bound, pp);
			if (types.isEmpty() || types.stream().anyMatch(t -> !ListOperations.INT.test(t) && !t.isNullType()))
				return false;
		}
		return true;
	}

	// whether the step of the slice is 0 (that raises ValueError)
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability stepZero(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression slice,
			ProgramPoint pp)
			throws SemanticException {
		if (!(slice instanceof TernaryExpression))
			return Satisfiability.NOT_SATISFIED;
		SymbolicExpression step = ((TernaryExpression) slice).getRight();
		Set<Type> types = analysis.getRuntimeTypesOf(state, step, pp);
		if (!types.isEmpty() && types.stream().allMatch(Type::isNullType))
			return Satisfiability.NOT_SATISFIED;
		return analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, step,
				new Constant(Int32Type.INSTANCE, 0, pp.getLocation()), ComparisonEq.INSTANCE, pp.getLocation()), pp);
	}

	/**
	 * Writes the element at the given index of the list
	 * ({@code list[index] = value}), raising {@code IndexError} if the index
	 * might be out of range, and {@code TypeError} if it might not be an
	 * {@code int}. The element is written exactly if the index is surely equal
	 * to the index of an element that the list surely has; otherwise, any of
	 * the elements might be written.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the list, the index and the value have
	 *                     been evaluated
	 * @param list     the list
	 * @param index    the index
	 * @param value    the value
	 * @param thrower  the statement writing the element
	 * @param raising  where the exceptions raised by the write are collected
	 *
	 * @return the state after the write, where the executions raising
	 *             exceptions have not been moved yet
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> write(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression index,
			SymbolicExpression value,
			Statement thrower,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference deref = new HeapDereference(listType, list, loc);
		Shape shape = shape(analysis, state, list, thrower);

		AnalysisState<A> result = state.bottomExecution();
		boolean typeError = false, indexError = false, any = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, index, thrower);
		if (types.isEmpty() || types.stream().anyMatch(t -> !ListOperations.INT.test(t)))
			typeError = true;
		// an unknown value (e.g. returned by an unknown function) might be an
		// int
		if (types.isEmpty() || types.stream().anyMatch(t -> ListOperations.INT.test(t) || t.isUntyped())) {
			int from = shape.length() == null ? 0 : -shape.length();
			int to = shape.length() == null ? shape.prefix() : shape.length();
			Integer exact = null;
			for (int c = from; c < to && exact == null; c++)
				if (analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, index,
						new Constant(Int32Type.INSTANCE, c, loc), ComparisonEq.INSTANCE, loc),
						thrower) == Satisfiability.SATISFIED)
					exact = c;
			if (exact != null) {
				int pos = exact < 0 ? shape.length() + exact : exact;
				result = result.lub(analysis.assign(state, new AccessChild(Untyped.INSTANCE, deref,
						new Constant(Int32Type.INSTANCE, pos, loc), loc), value, thrower));
			} else {
				Satisfiability in = Satisfiability.UNKNOWN;
				if (shape.length() != null) {
					Constant low = new Constant(Int32Type.INSTANCE, -shape.length(), loc);
					Constant high = new Constant(Int32Type.INSTANCE, shape.length(), loc);
					in = analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE,
							new BinaryExpression(BoolType.INSTANCE, index, low, ComparisonGe.INSTANCE, loc),
							new BinaryExpression(BoolType.INSTANCE, index, high, ComparisonLt.INSTANCE, loc),
							LogicalAnd.INSTANCE, loc), thrower);
				}
				any = in != Satisfiability.NOT_SATISFIED;
				indexError = in != Satisfiability.SATISFIED;
			}
		}

		if (any) {
			// any of the elements might be written
			AnalysisState<A> written = state;
			if (shape.elements() != null)
				for (SymbolicExpression field : shape.elements())
					written = analysis.assign(written, new AccessChild(Untyped.INSTANCE, deref, field, loc), value,
							thrower).lub(written);
			result = result.lub(written);
		}
		if (indexError)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.INDEX_ERROR);
		if (typeError)
			raising.raise(state, thrower.getCFG(), loc, thrower, LibrarySpecificationProvider.TYPE_ERROR);
		return result;
	}

	/**
	 * Yields any of the elements of the list (e.g. the one bound to the
	 * variable of a comprehension iterating over the list).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list
	 * @param pp       the program point reading the elements
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			ProgramPoint pp)
			throws SemanticException {
		return elements(analysis, state, list, PyClassType.lookup(LibrarySpecificationProvider.LIST), pp);
	}

	/**
	 * Yields any of the elements of the list or tuple, as
	 * {@link #elements(Analysis, AnalysisState, SymbolicExpression, ProgramPoint)},
	 * whose class is the given one.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param list     the list or tuple
	 * @param type     the class of {@code list}
	 * @param pp       the program point reading the elements
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			Type type,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Shape shape = shape(analysis, state, list, pp);
		if (shape.elements() == null)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		HeapDereference deref = new HeapDereference(type, list, loc);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression field : shape.elements())
			result = result.lub(
					analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, deref, field, loc), pp));
		return result;
	}
}
