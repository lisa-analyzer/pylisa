package it.unive.pylisa.libraries.lists;

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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Set;

/**
 * Native implementation of {@code list.__mul__(self, other)}: list repetition
 * ({@code [1] * 3}), that yields a new list (allocated at a location different
 * from the one of the expression, see {@link ListOperations#allocation}) with
 * the elements of {@code self} repeated {@code other} times. If the length of
 * {@code self} and {@code other} are known, the elements are copied at their
 * indices (up to {@value #MAX_EXACT} elements); otherwise, they are elements at
 * unknown indices of the new list (see {@link ListCreation#UNKNOWN_INDEX}),
 * that surely has an element if {@code self} surely has one and {@code other}
 * is surely positive (see {@link ListModel#NONEMPTY}). The new list is empty if
 * {@code self} is empty or {@code other} is not positive. {@code TypeError} is
 * raised if {@code other} might not be an {@code int}, and surely raised if it
 * is surely not one. This also handles the receivers whose type is unknown,
 * that might be lists: the result is then an unknown value (see
 * {@link ListOperations}).
 */
public class ListMul extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected ListMul(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static ListMul build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListMul(cfg, location, "__mul__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		return repeat(interprocedural, state, left, right, this, st, expressions);
	}

	/**
	 * The most elements of a repetition that are placed at their indices.
	 */
	static final int MAX_EXACT = 32;

	/**
	 * Computes {@code list * times}.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the operands have been evaluated
	 * @param list            the list
	 * @param times           the number of repetitions
	 * @param owner           the expression the new list belongs to
	 * @param st              the statement performing the repetition
	 * @param expressions     the statement store
	 *
	 * @return the state after the repetition
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> repeat(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression times,
			Expression owner,
			Statement st,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Set<Type> self = analysis.getRuntimeTypesOf(state, list, context, st);
		if (self.isEmpty() || !self.stream().allMatch(ListOperations.LIST))
			// the receiver might not be a list (e.g. its type is unknown): the
			// result is not tracked
			return ListOperations.unknownResult(interprocedural, state, times, ListOperations.INT, st);
		Set<Type> types = analysis.getRuntimeTypesOf(state, times, context, st);
		boolean unknown = types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (unknown || types.stream().anyMatch(ListOperations.INT))
			result = repetition(interprocedural, state, list, times, owner, st, expressions);
		if (unknown || !types.stream().allMatch(ListOperations.INT))
			raising.raise(state, st.getCFG(), owner.getLocation(), st, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}

	// the new list with the elements of list, repeated times times
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> repetition(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression list,
			SymbolicExpression times,
			Expression owner,
			Statement st,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = owner.getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		ListCreation creation = new ListCreation(owner.getCFG(), ListOperations.allocation(st, "repeat"));
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		owner.getMetaVariables().addAll(creation.getMetaVariables());
		// the operand is read after the allocation, that might have made its
		// allocation site weak (e.g. xs = xs * 2 in a loop)
		Integer length = ListModel.length(analysis, context, created, list, st);
		Satisfiability positive = analysis.satisfies(created,
				new it.unive.lisa.symbolic.value.BinaryExpression(BoolType.INSTANCE, times,
						new Constant(Int32Type.INSTANCE, 1, loc), ComparisonGe.INSTANCE, loc),
				context, st);
		// the number of repetitions, if it is known and the elements fit
		Integer count = null;
		if (length != null && length > 0)
			for (int c = 0; c * length <= MAX_EXACT && count == null; c++)
				if (analysis.satisfies(created,
						new it.unive.lisa.symbolic.value.BinaryExpression(BoolType.INSTANCE, times,
								new Constant(Int32Type.INSTANCE, c, loc), ComparisonEq.INSTANCE, loc),
						context,
						st) == Satisfiability.SATISFIED)
					count = c;
		Satisfiability nonNegative = analysis.satisfies(created,
				new it.unive.lisa.symbolic.value.BinaryExpression(BoolType.INSTANCE, times,
						new Constant(Int32Type.INSTANCE, 0, loc), ComparisonGe.INSTANCE, loc),
				context, st);
		boolean nonEmpty = positive == Satisfiability.SATISFIED
				&& ListOperations.nonEmpty(analysis, context, created, list, st);

		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression repeated : created.getExecutionExpressions()) {
			HeapDereference target = new HeapDereference(listType, repeated, loc);
			AnalysisState<A> filled = created;
			if (positive == Satisfiability.NOT_SATISFIED || (length != null && length == 0))
				// no element
				;
			else if (count != null) {
				// the elements at known indices
				for (int r = 0; r < count; r++)
					for (int i = 0; i < length; i++)
						filled = ListOperations.copy(analysis, context, filled, created, list, i, target,
								r * length + i, st);
				filled = ListModel.setLength(analysis, context, filled, repeated,
						new Constant(Int32Type.INSTANCE, count * length, loc), st);
			} else {
				// the elements at unknown indices
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, target,
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				AnalysisState<A> unknowns = filled.bottomExecution();
				for (SymbolicExpression element : ListOperations.elements(analysis, context, created, list, st))
					unknowns = unknowns.lub(analysis.assign(filled, unknown, element, context, st));
				if (!unknowns.getExecutionState().isBottom())
					filled = unknowns;
				if (nonEmpty)
					filled = ListModel.setNonEmpty(analysis, context, filled, repeated, true, st);
				// len(list) * times elements, if times is not negative (for a
				// list of one element, the number of repetitions itself, e.g.
				// [0] * n has n elements)
				if (nonNegative == Satisfiability.SATISFIED)
					filled = ListModel.setLength(analysis, context, filled, repeated,
							length != null && length == 1 ? times
									: new it.unive.lisa.symbolic.value.BinaryExpression(Int32Type.INSTANCE,
											ListModel.lengthOf(list, loc), times,
											NumericNonOverflowingMul.INSTANCE, loc),
							st);
				else
					filled = ListModel.unknownLength(analysis, context, filled, repeated, st);
			}
			result = result.lub(filled.withExecutionExpression(repeated));
		}
		return result;
	}
}
