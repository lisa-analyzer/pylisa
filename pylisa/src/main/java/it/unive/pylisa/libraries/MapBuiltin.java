package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyValueCall;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;

/**
 * Native implementation of {@code map(function, *iterables)}: a new object of
 * the class {@code Map}, whose child {@value #FUNCTION} is the function and
 * whose child {@value #ITERABLES} is the list collecting the iterables (see
 * {@code PyAssigningStrategy}). As map objects are lazy, the function is called
 * when the object is iterated (see {@link #elements}), on an element of each
 * iterable, and the number of elements is not tracked (so that iterating the
 * same object again, that yields no element in python, is also covered).
 * Unpacking a map object of a single list or tuple of known length calls the
 * function on its elements, in order (see {@link #positional}).
 * {@code TypeError} is raised if an iterable might not be iterable (see
 * {@link Iteration#iterable}), or if there are no iterables.
 */
public class MapBuiltin extends PyNative {

	/**
	 * The child of map objects holding the function.
	 */
	public static final String FUNCTION = "$function";

	/**
	 * The child of map objects holding the list of the iterables.
	 */
	public static final String ITERABLES = "$iterables";

	protected MapBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "map", params);
	}

	public static MapBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new MapBuiltin(cfg, location, exprs);
	}

	/**
	 * Whether the values of the given type are map objects.
	 *
	 * @param t the type
	 *
	 * @return whether the values are map objects
	 */
	public static boolean isMap(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.MAP);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression function = args[0], iterables = args[1];
		// the iterables, if their number is known
		List<SymbolicExpression> known = ListModel.exactElements(analysis, context, state, iterables, st);
		Satisfiability typed = known == null ? Satisfiability.UNKNOWN
				: known.isEmpty() ? Satisfiability.NOT_SATISFIED : Satisfiability.SATISFIED;
		if (known != null)
			for (SymbolicExpression iterable : known)
				typed = typed.and(Iteration.iterable(analysis, context, state, iterable, st));

		AnalysisState<A> created = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			Type type = PyClassType.lookup(LibrarySpecificationProvider.MAP);
			ReferenceType reftype = new ReferenceType(type);
			// a location of its own, as the list of the iterables is allocated
			// at the location of the call
			CodeLocation mapLoc = new DerivedLocation(loc, "map");
			MemoryAllocation alloc = new MemoryAllocation(type, mapLoc, false);
			InstrumentedReceiver map = new InstrumentedReceiver(reftype, false, mapLoc);
			created = analysis.assign(Allocations.allocate(analysis, context, state, alloc, st), map,
					new HeapReference(reftype, alloc, mapLoc), context, st);
			HeapDereference deref = new HeapDereference(type, map, mapLoc);
			created = analysis.assign(created, child(deref, FUNCTION, mapLoc), function, context, st);
			created = analysis.assign(created, child(deref, ITERABLES, mapLoc), iterables, context, st);
			getMetaVariables().add(map);
			created = compute(analysis, context, created, map);
		}
		return typeChecked(analysis, context, state, typed, created);
	}

	/**
	 * Yields any of the elements of a map object, as done when iterating it:
	 * the result of calling the function on an element of each iterable, read
	 * from their current elements (see {@link Iteration#elements}). The element
	 * is not known, and any exception might be raised, if the number of the
	 * iterables is not known.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the current state
	 * @param map             the map object
	 * @param pp              the program point iterating the object
	 * @param metas           where the meta variables of the calls are added
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression map,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = pp.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.MAP), map, loc);
		ExpressionSet functions = analysis.smallStepSemantics(state, child(deref, FUNCTION, loc), context, pp)
				.getExecutionExpressions();
		AnalysisState<A> lists = analysis.smallStepSemantics(state, child(deref, ITERABLES, loc), context, pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression list : lists.getExecutionExpressions()) {
			List<SymbolicExpression> iterables = ListModel.exactElements(analysis, context, lists, list, pp);
			if (iterables == null) {
				result = result.lub(uncalled(analysis, context, state, pp));
				continue;
			}
			AnalysisState<A> current = lists;
			ExpressionSet[] args = new ExpressionSet[iterables.size()];
			for (int i = 0; i < args.length; i++) {
				current = Iteration.elements(interprocedural, current, null, iterables.get(i), pp, metas);
				args[i] = current.getExecutionExpressions();
			}
			result = result.lub(PyValueCall.call(interprocedural, current, functions, args, pp, "map",
					new StatementStore<>(current), metas));
		}
		return result;
	}

	/**
	 * Yields the elements of a map object of a single list or tuple of known
	 * length, in order, as done when unpacking it: the results of calling the
	 * function on each element of the iterable, read from its current elements.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the current state
	 * @param map             the map object
	 * @param pp              the program point unpacking the object
	 * @param metas           where the meta variables of the calls are added
	 *
	 * @return the state after the calls, and the values of each element, or
	 *             {@code null} if the elements are not known in order
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Positional<A> positional(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression map,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = pp.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.MAP), map, loc);
		ExpressionSet functions = analysis.smallStepSemantics(state, child(deref, FUNCTION, loc), context, pp)
				.getExecutionExpressions();
		AnalysisState<A> lists = analysis.smallStepSemantics(state, child(deref, ITERABLES, loc), context, pp);
		if (lists.getExecutionExpressions().size() != 1)
			return null;
		List<SymbolicExpression> iterables = ListModel.exactElements(analysis, context, lists,
				lists.getExecutionExpressions().iterator().next(), pp);
		if (iterables == null || iterables.size() != 1)
			return null;
		SymbolicExpression iterable = iterables.get(0);
		Set<Type> types = analysis.getRuntimeTypesOf(lists, iterable, context, pp);
		Type type = types.size() == 1 ? types.iterator().next() : null;
		if (type == null || !(ListModel.isList(type) || Unpacking.isTuple(type)))
			return null;
		Integer length = ListModel.length(analysis, context, lists, iterable, pp);
		if (length == null)
			return null;

		HeapDereference source = new HeapDereference(type.asPointerType().getInnerType(), iterable, loc);
		AnalysisState<A> current = lists;
		List<ExpressionSet> values = new ArrayList<>();
		for (int i = 0; i < length; i++) {
			ExpressionSet element = analysis.smallStepSemantics(current,
					new AccessChild(Untyped.INSTANCE, source, new Constant(Int32Type.INSTANCE, i, loc), loc), context,
					pp)
					.getExecutionExpressions();
			// each call has a location of its own, so that its result is not
			// overwritten by the next ones
			current = PyValueCall.call(interprocedural, current, functions, new ExpressionSet[] { element }, pp,
					"map" + i, new StatementStore<>(current), metas);
			values.add(current.getExecutionExpressions());
		}
		return new Positional<>(current, values);
	}

	/**
	 * The elements of a map object in order (see {@link #positional}).
	 *
	 * @param <A>    the kind of the analysis state
	 * @param state  the state after the calls
	 * @param values the values of each element
	 */
	public record Positional<A extends AbstractLattice<A>>(
			AnalysisState<A> state,
			List<ExpressionSet> values) {
	}

	/**
	 * Yields any of the elements of a map object when the function cannot be
	 * called (that is, without the interprocedural analysis, see
	 * {@link Iteration#elements}): an unknown value, and any exception might be
	 * raised by the function.
	 *
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param pp       the program point iterating the object
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> uncalled(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		raising.raise(state, pp.getCFG(), loc, pp, LibrarySpecificationProvider.EXCEPTION);
		return raising.moveFrom(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), context, pp));
	}

	private static AccessChild child(
			HeapDereference map,
			String name,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, map, new Variable(Untyped.INSTANCE, name, loc), loc);
	}
}
