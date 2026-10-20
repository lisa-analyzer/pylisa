package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
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
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collection;
import org.apache.commons.lang3.StringUtils;

public class TupleCreation extends NaryExpression {

	public TupleCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "tuple", values);
	}

	/**
	 * The location where the tuple is allocated. It differs from the location
	 * of the tuple, that is also the one of its first element when there are no
	 * parentheses (e.g. {@code [1]} or {@code P()} in {@code [1], 2}), as
	 * memory allocations are identified by their location (see
	 * {@link DerivedLocation}).
	 *
	 * @return the location
	 */
	public CodeLocation allocation() {
		Expression[] subs = getSubExpressions();
		if (subs.length > 0 && subs[0] instanceof TupleCreation && subs[0].getLocation().equals(getLocation()))
			// e.g. (1, 2), 3
			return new DerivedLocation(((TupleCreation) subs[0]).allocation(), "tuple");
		return new DerivedLocation(getLocation(), "tuple");
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		Expression[] subs = getSubExpressions();
		if (subs.length == 1)
			return "(" + StringUtils.join(subs, ", ") + ")";
		return super.toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		// a tuple creation is also created when parsing expressions between
		// parentheses
		// if we only have one sub-expression, we assume to be in that case
		if (params.length == 1)
			return state;

		return create(analysis, state, this, allocation(), params, getMetaVariables());
	}

	/**
	 * Creates a tuple with the given elements: a new object is allocated, and
	 * the receiver pointing to it (an {@link InstrumentedReceiver}) is the
	 * value of the tuple. The elements are the children {@code 0, 1, ...} of
	 * the object, and since tuples are immutable their number is also stored,
	 * in the {@code length} child.
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param state    the state where the elements have been computed
	 * @param pp       the statement creating the tuple
	 * @param location the location of the tuple
	 * @param params   the elements
	 * @param metas    where the receiver is added, as it is a meta variable
	 *                     that the statement using the tuple must forget
	 *
	 * @return the state where the tuple has been created
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Statement pp,
			CodeLocation location,
			ExpressionSet[] params,
			Collection<Identifier> metas)
			throws SemanticException {
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);
		ReferenceType reftype = new ReferenceType(tupleType);
		HeapReference ref = new HeapReference(reftype, new MemoryAllocation(tupleType, location), location);
		InstrumentedReceiver tuple = new InstrumentedReceiver(reftype, false, location);
		AnalysisState<A> current = analysis.assign(state, tuple, ref, pp);
		HeapDereference deref = new HeapDereference(tupleType, tuple, location);

		for (int i = 0; i < params.length; i++) {
			AccessChild element = new AccessChild(Untyped.INSTANCE, deref,
					new Constant(Int32Type.INSTANCE, i, location),
					location);
			AnalysisState<A> next = state.bottomExecution();
			for (SymbolicExpression init : params[i])
				next = next.lub(analysis.assign(current, element, init, pp));
			current = next;
		}

		// tuples are immutable, so the element count fixed at creation time
		// never becomes stale: it is tracked as a "length" field so that
		// SequenceGetItem can raise IndexError on out-of-bounds accesses
		AccessChild length = new AccessChild(Untyped.INSTANCE, deref,
				new Variable(Int32Type.INSTANCE, "length", location), location);
		current = analysis.assign(current, length, new Constant(Int32Type.INSTANCE, params.length, location), pp);

		metas.add(tuple);
		return analysis.smallStepSemantics(current, tuple, pp);
	}
}