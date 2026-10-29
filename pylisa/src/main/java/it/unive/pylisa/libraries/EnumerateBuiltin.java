package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
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
import java.util.Collection;

/**
 * Native implementation of {@code enumerate(iterable, start=0)}: a new object
 * of the class {@code Enumerate}, whose child {@value #ITERABLE} is the
 * iterable. As enumerate objects are lazy, the elements of the iterable are
 * read when the object is iterated (see {@link #elements}): each element is a
 * tuple with an {@code int} (whose value is not tracked) and an element of the
 * iterable, and the number of elements is not tracked. {@code TypeError} is
 * raised if the iterable might not be iterable (see
 * {@link Iteration#iterable}), or if {@code start} might not be an {@code int}.
 */
public class EnumerateBuiltin extends PyNative {

	/**
	 * The child of enumerate objects holding the iterable.
	 */
	public static final String ITERABLE = "$iterable";

	protected EnumerateBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "enumerate", params);
	}

	public static EnumerateBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new EnumerateBuiltin(cfg, location, exprs);
	}

	/**
	 * Whether the values of the given type are enumerate objects.
	 *
	 * @param t the type
	 *
	 * @return whether the values are enumerate objects
	 */
	public static boolean isEnumerate(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.ENUMERATE);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression iterable = args[0], start = args[1];
		Satisfiability typed = Iteration.iterable(analysis, context, state, iterable, st);
		if (!missing(start))
			typed = typed.and(hasType(analysis, context, state, start, INT));

		AnalysisState<A> created = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED) {
			Type type = PyClassType.lookup(LibrarySpecificationProvider.ENUMERATE);
			ReferenceType reftype = new ReferenceType(type);
			MemoryAllocation alloc = new MemoryAllocation(type, loc, false);
			InstrumentedReceiver enumerate = new InstrumentedReceiver(reftype, false, loc);
			created = analysis.assign(Allocations.allocate(analysis, context, state, alloc, st), enumerate,
					new HeapReference(reftype, alloc, loc), context, st);
			created = analysis.assign(created, child(new HeapDereference(type, enumerate, loc), loc), iterable, context,
					st);
			getMetaVariables().add(enumerate);
			created = compute(analysis, context, created, enumerate);
		}
		return typeChecked(analysis, context, state, typed, created);
	}

	/**
	 * Yields any of the elements of an enumerate object, as done when iterating
	 * it: a new tuple with an {@code int} and an element of the iterable, read
	 * from its current elements (see {@link Iteration#elements}).
	 *
	 * @param analysis  the analysis
	 * @param context   the interprocedural context where the computation
	 *                      happens
	 * @param state     the current state
	 * @param enumerate the enumerate object
	 * @param pp        the program point iterating the object
	 * @param metas     where the meta variables of the tuples are added
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression enumerate,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type type = PyClassType.lookup(LibrarySpecificationProvider.ENUMERATE);
		AnalysisState<A> iterables = analysis.smallStepSemantics(state,
				child(new HeapDereference(type, enumerate, loc), loc), context, pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression iterable : iterables.getExecutionExpressions()) {
			AnalysisState<A> count = analysis.smallStepSemantics(iterables, new PushAny(Int32Type.INSTANCE, loc),
					context, pp);
			AnalysisState<A> element = Iteration.elements(analysis, context, count, null, iterable, pp, metas);
			ExpressionSet[] values = { count.getExecutionExpressions(), element.getExecutionExpressions() };
			result = result
					.lub(TupleCreation.create(analysis, context, element, pp, new DerivedLocation(loc, "enumerate"),
							values, metas));
		}
		return result;
	}

	private static AccessChild child(
			HeapDereference enumerate,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, enumerate, new Variable(Untyped.INSTANCE, ITERABLE, loc), loc);
	}
}
