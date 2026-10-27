package it.unive.pylisa.libraries.itertools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyValueCall;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collection;
import java.util.List;
import java.util.Set;

/**
 * The iterators of {@code itertools}: objects of the class {@value #CLASS},
 * whose child {@value #KIND} is the function creating them (e.g.
 * {@value #COUNT}), and whose other children are its arguments. As the
 * iterators are lazy, their elements are computed when they are iterated (see
 * {@link #elements}), from the current elements of the iterables they hold, and
 * their number is not tracked (so that iterating them again, that yields no
 * element in python, is also covered):
 * <ul>
 * <li>{@code count(start, step)}: numbers, {@code int}s if {@code start} and
 * {@code step} are, {@code float}s otherwise;</li>
 * <li>{@code cycle(iterable)}, {@code compress(data, selectors)} (the elements
 * of {@code data}) and the groups of {@code groupby}: the elements of the
 * iterable;</li>
 * <li>{@code repeat(object, times)}: {@code object};</li>
 * <li>{@code chain(*iterables)}: the elements of the iterables;</li>
 * <li>{@code permutations(iterable, r)} and {@code combinations(iterable, r)}:
 * tuples of {@code r} elements of the iterable (of unknown length if {@code r}
 * is not known);</li>
 * <li>{@code product(*iterables, repeat=1)}: tuples with an element of each
 * iterable, {@code repeat} times (of unknown length if the number of the
 * iterables, or {@code repeat}, are not known);</li>
 * <li>{@code groupby(iterable, key=None)}: tuples of the key of an element
 * ({@code key(element)}, or the element itself) and of the group, an iterator
 * (of kind {@value #GROUPER}) over the elements of the iterable.</li>
 * </ul>
 */
public final class ItertoolsModel {

	/**
	 * The class of the iterators: a single one, as each class of the libraries
	 * is a type of the program, and the values whose type is not known might be
	 * of any of them (so that a class for each iterator would make the analysis
	 * of such values slower).
	 */
	static final String CLASS = "Itertool";

	/**
	 * The child of the iterators holding their kind (e.g. {@value #COUNT}).
	 */
	static final String KIND = "$kind";

	// the kinds of the iterators
	static final String COUNT = "count", CYCLE = "cycle", REPEAT = "repeat", CHAIN = "chain",
			COMPRESS = "compress", PERMUTATIONS = "permutations", COMBINATIONS = "combinations",
			PRODUCT = "product", GROUPBY = "groupby", GROUPER = "grouper";

	private static final List<String> KINDS = List.of(COUNT, CYCLE, REPEAT, CHAIN, COMPRESS, PERMUTATIONS,
			COMBINATIONS, PRODUCT, GROUPBY, GROUPER);

	// the children of the iterators holding the arguments
	static final String START = "$start", STEP = "$step", ITERABLE = "$iterable", OBJECT = "$object",
			ITERABLES = "$iterables", R = "$r", REPEATS = "$repeat", KEY = "$key";

	// the largest length of the tuples whose elements are tracked
	private static final int MAX_LENGTH = 8;

	private ItertoolsModel() {
	}

	/**
	 * Whether the values of the given type are iterators of {@code itertools}.
	 *
	 * @param t the type
	 *
	 * @return whether the values are such iterators
	 */
	public static boolean isIterator(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName().equals(CLASS);
	}

	/**
	 * Whether the given iterators of {@code itertools} are never exhausted
	 * (that is, they are all created by {@code count}).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param iterator the iterator
	 * @param pp       the program point using the iterator
	 *
	 * @return whether the iterators are infinite
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean infinite(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterator,
			Statement pp)
			throws SemanticException {
		HeapDereference deref = new HeapDereference(PyClassType.lookup(CLASS), iterator, pp.getLocation());
		return kinds(analysis, state, deref, pp).equals(Set.of(COUNT));
	}

	// the kinds the iterator might have
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<String> kinds(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> values = analysis.smallStepSemantics(state, child(deref, KIND, loc), pp);
		Set<String> kinds = new java.util.TreeSet<>();
		for (SymbolicExpression value : values.getExecutionExpressions())
			for (String kind : KINDS)
				if (analysis.satisfies(values, new BinaryExpression(BoolType.INSTANCE, value,
						new Constant(StringType.INSTANCE, kind, loc), ComparisonEq.INSTANCE, loc),
						pp) != Satisfiability.NOT_SATISFIED)
					kinds.add(kind);
		return kinds;
	}

	/**
	 * Creates an iterator of the given kind, with the given children.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the children have been computed
	 * @param pp       the statement creating the iterator
	 * @param kind     the kind of the iterator
	 * @param names    the names of the children
	 * @param children the values of the children
	 * @param metas    where the receiver of the iterator is added
	 *
	 * @return the state whose computed expression is the iterator
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Statement pp,
			String kind,
			String[] names,
			SymbolicExpression[] children,
			Collection<Identifier> metas)
			throws SemanticException {
		// a location of its own, as the list of the iterables of some of them
		// is allocated at the location of the call
		CodeLocation loc = new DerivedLocation(pp.getLocation(), kind);
		Type type = PyClassType.lookup(CLASS);
		ReferenceType reftype = new ReferenceType(type);
		MemoryAllocation alloc = new MemoryAllocation(type, loc, false);
		InstrumentedReceiver it = new InstrumentedReceiver(reftype, false, loc);
		AnalysisState<A> current = analysis.assign(Allocations.allocate(analysis, state, alloc, pp), it,
				new HeapReference(reftype, alloc, loc), pp);
		HeapDereference deref = new HeapDereference(type, it, loc);
		current = analysis.assign(current, child(deref, KIND, loc), new Constant(StringType.INSTANCE, kind, loc), pp);
		for (int i = 0; i < names.length; i++)
			current = analysis.assign(current, child(deref, names[i], loc), children[i], pp);
		metas.add(it);
		return analysis.smallStepSemantics(current, it, pp);
	}

	/**
	 * Yields any of the elements of an iterator of {@code itertools}, as done
	 * when iterating it. The keys of {@code groupby} are computed by calling
	 * its key function, if the interprocedural analysis is given: otherwise,
	 * they are unknown, and any exception might be raised.
	 *
	 * @param analysis        the analysis
	 * @param interprocedural the interprocedural analysis, or {@code null}
	 * @param state           the current state
	 * @param iterator        the iterator
	 * @param pp              the program point iterating the iterator
	 * @param metas           where the meta variables of the elements are added
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression iterator,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> result = state.bottomExecution();
		if (analysis.getRuntimeTypesOf(state, iterator, pp).stream().noneMatch(ItertoolsModel::isIterator))
			return result;
		HeapDereference deref = new HeapDereference(PyClassType.lookup(CLASS), iterator, loc);
		for (String kind : kinds(analysis, state, deref, pp))
			switch (kind) {
			case COUNT:
				result = result.lub(count(analysis, state, deref, pp));
				break;
			case CYCLE:
			case COMPRESS:
			case GROUPER:
				result = result.lub(of(analysis, interprocedural, state, deref, ITERABLE, pp, metas));
				break;
			case REPEAT:
				result = result.lub(analysis.smallStepSemantics(state, child(deref, OBJECT, loc), pp));
				break;
			case CHAIN:
				for (List<SymbolicExpression> iterables : iterables(analysis, state, deref, pp))
					if (iterables == null)
						result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc),
								pp));
					else
						for (SymbolicExpression iterable : iterables)
							result = result.lub(elementsOf(analysis, interprocedural, state, iterable, pp, metas));
				break;
			case PERMUTATIONS:
			case COMBINATIONS:
				result = result.lub(tuples(analysis, interprocedural, state, deref, pp, metas));
				break;
			case PRODUCT:
				result = result.lub(product(analysis, interprocedural, state, deref, pp, metas));
				break;
			case GROUPBY:
				result = result.lub(groups(analysis, interprocedural, state, deref, pp, metas));
				break;
			default:
				break;
			}
		return result;
	}

	// the numbers of count
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> count(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		boolean ints = true;
		for (String name : new String[] { START, STEP }) {
			AnalysisState<A> values = analysis.smallStepSemantics(state, child(deref, name, loc), pp);
			for (SymbolicExpression value : values.getExecutionExpressions())
				for (Type t : analysis.getRuntimeTypesOf(values, value, pp))
					ints &= (t.isNumericType() && t.asNumericType().isIntegral()) || t.isBooleanType();
		}
		return analysis.smallStepSemantics(state,
				new PushAny(ints ? Int32Type.INSTANCE : Float32Type.INSTANCE, loc), pp);
	}

	// the elements of the iterable held by the given child
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> of(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			HeapDereference deref,
			String name,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		AnalysisState<A> iterables = analysis.smallStepSemantics(state, child(deref, name, pp.getLocation()), pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression iterable : iterables.getExecutionExpressions())
			result = result.lub(elementsOf(analysis, interprocedural, iterables, iterable, pp, metas));
		return result;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elementsOf(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression iterable,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		return interprocedural != null ? Iteration.elements(interprocedural, state, null, iterable, pp, metas)
				: Iteration.elements(analysis, state, null, iterable, pp, metas);
	}

	// the lists of the iterables of chain and product (null if they are not
	// known)
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<List<SymbolicExpression>> iterables(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp)
			throws SemanticException {
		AnalysisState<A> lists = analysis.smallStepSemantics(state, child(deref, ITERABLES, pp.getLocation()), pp);
		List<List<SymbolicExpression>> result = new java.util.ArrayList<>();
		for (SymbolicExpression list : lists.getExecutionExpressions())
			result.add(ListModel.exactElements(analysis, lists, list, pp));
		return result;
	}

	// the value of the given child, if it is surely a non-negative int at
	// most MAX_LENGTH, or null
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Integer known(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			String name,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> values = analysis.smallStepSemantics(state, child(deref, name, loc), pp);
		if (values.getExecutionExpressions().size() != 1)
			return null;
		SymbolicExpression value = values.getExecutionExpressions().iterator().next();
		for (int k = 0; k <= MAX_LENGTH; k++)
			if (analysis.satisfies(values, new BinaryExpression(BoolType.INSTANCE, value,
					new Constant(Int32Type.INSTANCE, k, loc), ComparisonEq.INSTANCE, loc),
					pp) == Satisfiability.SATISFIED)
				return k;
		return null;
	}

	// the tuples of permutations and combinations
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> tuples(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		AnalysisState<A> elements = of(analysis, interprocedural, state, deref, ITERABLE, pp, metas);
		Integer r = known(analysis, state, deref, R, pp);
		if (r == null)
			return unknownTuple(analysis, elements, pp, metas);
		ExpressionSet[] values = new ExpressionSet[r];
		for (int i = 0; i < r; i++)
			values[i] = elements.getExecutionExpressions();
		if (r > 0 && elements.getExecutionState().isBottom())
			// no elements: no tuples
			return state.bottomExecution();
		return TupleCreation.create(analysis, elements, pp, location(pp), values, metas);
	}

	// the tuples of product
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> product(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		Integer repeat = known(analysis, state, deref, REPEATS, pp);
		AnalysisState<A> result = state.bottomExecution();
		for (List<SymbolicExpression> iterables : iterables(analysis, state, deref, pp)) {
			if (iterables == null || repeat == null || iterables.size() * repeat > MAX_LENGTH) {
				result = result.lub(unknownTuple(analysis, state, pp, metas));
				continue;
			}
			AnalysisState<A> current = state;
			ExpressionSet[] values = new ExpressionSet[iterables.size() * repeat];
			boolean empty = false;
			for (int i = 0; i < iterables.size(); i++) {
				current = elementsOf(analysis, interprocedural, current, iterables.get(i), pp, metas);
				empty |= current.getExecutionState().isBottom();
				for (int j = 0; j < repeat; j++)
					values[j * iterables.size() + i] = current.getExecutionExpressions();
			}
			if (!empty)
				result = result.lub(TupleCreation.create(analysis, current, pp, location(pp), values, metas));
		}
		return result;
	}

	// the tuples of groupby: the key, and the group
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> groups(
			Analysis<A, D> analysis,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			HeapDereference deref,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		AnalysisState<A> iterables = analysis.smallStepSemantics(state, child(deref, ITERABLE, loc), pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression iterable : iterables.getExecutionExpressions()) {
			AnalysisState<A> elements = elementsOf(analysis, interprocedural, iterables, iterable, pp, metas);
			if (elements.getExecutionState().isBottom())
				continue;
			// the keys
			AnalysisState<A> keys = elements.bottomExecution();
			AnalysisState<A> functions = analysis.smallStepSemantics(elements, child(deref, KEY, loc), pp);
			for (SymbolicExpression key : functions.getExecutionExpressions())
				if (analysis.getRuntimeTypesOf(functions, key, pp).stream().allMatch(Type::isNullType))
					keys = keys.lub(elements);
				else if (interprocedural == null)
					keys = keys.lub(analysis.smallStepSemantics(elements, new PushAny(Untyped.INSTANCE, loc), pp))
							.lub(PyExceptions.raise(analysis, elements, pp.getCFG(), loc, pp,
									LibrarySpecificationProvider.EXCEPTION));
				else
					keys = keys.lub(PyValueCall.call(interprocedural, functions, new ExpressionSet(key),
							new ExpressionSet[] { elements.getExecutionExpressions() }, pp, "key",
							new StatementStore<>(functions), metas));
			// the group: an iterator over the elements of the iterable
			AnalysisState<A> group = create(analysis, keys, pp, GROUPER, new String[] { ITERABLE },
					new SymbolicExpression[] { iterable }, metas);
			ExpressionSet[] values = { keys.getExecutionExpressions(), group.getExecutionExpressions() };
			result = result.lub(TupleCreation.create(analysis, group, pp, location(pp), values, metas));
		}
		return result;
	}

	// a tuple whose length and elements are not known
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknownTuple(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = location(pp);
		AnalysisState<A> created = TupleCreation.create(analysis, state, pp, loc, new ExpressionSet[0], metas);
		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression tuple : created.getExecutionExpressions())
			result = result.lub(TupleCreation.unknownLength(analysis, created, tuple, pp, loc));
		return result;
	}

	private static CodeLocation location(
			Statement pp) {
		return new DerivedLocation(pp.getLocation(), "itertools");
	}

	static AccessChild child(
			HeapDereference deref,
			String name,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, name, loc), loc);
	}
}
