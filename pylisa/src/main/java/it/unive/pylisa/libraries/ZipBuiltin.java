package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
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
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collection;
import java.util.List;

/**
 * Native implementation of {@code zip(*iterables, strict=False)}: a new object
 * of the class {@code Zip}, whose child {@value #ITERABLES} is the list
 * collecting the iterables (see {@code PyAssigningStrategy}). As zip objects
 * are lazy, the elements of the iterables are read when the object is iterated
 * (see {@link #elements}): each element is a tuple with an element of each
 * iterable, and the number of elements is not tracked (so that iterating the
 * same object again, that yields no element in python, is also covered).
 * {@code TypeError} is raised if an iterable might not be iterable (see
 * {@link Iteration#iterable}), and {@code ValueError} might be raised if
 * {@code strict} might hold (when the lengths of the iterables differ): here,
 * when the object is created rather than while iterating it.
 */
public class ZipBuiltin extends PyNative {

	/**
	 * The child of zip objects holding the list of the iterables.
	 */
	public static final String ITERABLES = "$iterables";

	protected ZipBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "zip", params);
	}

	public static ZipBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ZipBuiltin(cfg, location, exprs);
	}

	/**
	 * Whether the values of the given type are zip objects.
	 *
	 * @param t the type
	 *
	 * @return whether the values are zip objects
	 */
	public static boolean isZip(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.ZIP);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression iterables = args[0], strict = args[1];
		// the iterables, if their number is known
		List<SymbolicExpression> known = ListModel.exactElements(analysis, state, iterables, st);
		Satisfiability typed = known == null ? Satisfiability.UNKNOWN : Satisfiability.SATISFIED;
		if (known != null)
			for (SymbolicExpression iterable : known)
				typed = typed.and(Iteration.iterable(analysis, state, iterable, st));

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> created = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			Type type = PyClassType.lookup(LibrarySpecificationProvider.ZIP);
			ReferenceType reftype = new ReferenceType(type);
			// a location of its own, as the list of the iterables is allocated
			// at the location of the call
			CodeLocation zipLoc = new DerivedLocation(loc, "zip");
			MemoryAllocation alloc = new MemoryAllocation(type, zipLoc, false);
			InstrumentedReceiver zip = new InstrumentedReceiver(reftype, false, zipLoc);
			created = analysis.assign(Allocations.allocate(analysis, state, alloc, st), zip,
					new HeapReference(reftype, alloc, zipLoc), st);
			created = analysis.assign(created, child(new HeapDereference(type, zip, zipLoc), zipLoc), iterables,
					st);
			getMetaVariables().add(zip);
			created = compute(analysis, created, zip);
			if (analysis.satisfies(state, strict, st) != Satisfiability.NOT_SATISFIED)
				// strict, with iterables of different lengths
				raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
		}
		return raising.moveFrom(typeChecked(state, typed, created, raising));
	}

	/**
	 * Yields any of the elements of a zip object, as done when iterating it: a
	 * new tuple with an element of each iterable, read from their current
	 * elements (see {@link Iteration#elements}). The element is not known if
	 * the number of the iterables is not known, and there is none if there are
	 * no iterables (as for {@code zip()}) or one of them has no element.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param zip      the zip object
	 * @param pp       the program point iterating the object
	 * @param metas    where the meta variables of the tuples are added
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression zip,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type type = PyClassType.lookup(LibrarySpecificationProvider.ZIP);
		AnalysisState<A> lists = analysis.smallStepSemantics(state, child(new HeapDereference(type, zip, loc), loc),
				pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression list : lists.getExecutionExpressions()) {
			List<SymbolicExpression> iterables = ListModel.exactElements(analysis, lists, list, pp);
			if (iterables == null) {
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp));
				continue;
			}
			if (iterables.isEmpty())
				// zip() has no elements
				continue;
			AnalysisState<A> current = lists;
			ExpressionSet[] elements = new ExpressionSet[iterables.size()];
			for (int i = 0; i < elements.length; i++) {
				current = Iteration.elements(analysis, current, null, iterables.get(i), pp, metas);
				elements[i] = current.getExecutionExpressions();
			}
			result = result.lub(
					TupleCreation.create(analysis, current, pp, new DerivedLocation(loc, "zip"), elements, metas));
		}
		return result;
	}

	private static AccessChild child(
			HeapDereference zip,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, zip, new Variable(Untyped.INSTANCE, ITERABLES, loc), loc);
	}
}
