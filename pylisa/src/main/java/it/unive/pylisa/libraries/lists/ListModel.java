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
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
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
			for (SymbolicExpression field : fields)
				if (field instanceof Variable && ((Variable) field).getName().equals(ListCreation.UNKNOWN_INDEX))
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
			int surely = site.isWeak() ? 0 : k;
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
	 *
	 * @return the state after the read
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> read(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression index,
			Statement thrower)
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
				indexError = true;
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
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.INDEX_ERROR));
		if (typeError)
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.TYPE_ERROR));
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
	 * {@code None}).
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the list and the slice have been
	 *                            evaluated
	 * @param list            the list
	 * @param slice           the slice
	 * @param owner           the expression the new list belongs to
	 * @param thrower         the statement reading the slice
	 *
	 * @return the state after the read
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> slice(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression slice,
			Expression owner,
			Statement thrower)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = owner.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference source = new HeapDereference(listType, list, loc);
		Shape shape = shape(analysis, state, list, owner);

		boolean typeError = !(slice instanceof TernaryExpression);
		if (slice instanceof TernaryExpression) {
			TernaryExpression bounds = (TernaryExpression) slice;
			for (SymbolicExpression bound : new SymbolicExpression[] { bounds.getLeft(), bounds.getMiddle(),
					bounds.getRight() }) {
				Set<Type> types = analysis.getRuntimeTypesOf(state, bound, owner);
				if (types.isEmpty() || types.stream().anyMatch(t -> !ListOperations.INT.test(t) && !t.isNullType()))
					typeError = true;
			}
		}

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
		if (typeError)
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
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
	 *
	 * @return the state after the write
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> write(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression index,
			SymbolicExpression value,
			Statement thrower)
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
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.INDEX_ERROR));
		if (typeError)
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.TYPE_ERROR));
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
		CodeLocation loc = pp.getLocation();
		Shape shape = shape(analysis, state, list, pp);
		if (shape.elements() == null)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.LIST), list,
				loc);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression field : shape.elements())
			result = result.lub(
					analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, deref, field, loc), pp));
		return result;
	}
}
