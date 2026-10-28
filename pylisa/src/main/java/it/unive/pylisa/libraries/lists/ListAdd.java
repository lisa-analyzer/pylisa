package it.unive.pylisa.libraries.lists;

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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.HashSet;
import java.util.Set;

/**
 * Native implementation of {@code list.__add__(self, other)}: list
 * concatenation ({@code [1] + [2]}), that yields a new list (allocated at a
 * location different from the one of the expression, see
 * {@link DerivedLocation}). If the length of {@code self} is known (see
 * {@link ListModel#length}), its elements are copied at their indices, and so
 * are the ones of {@code other}, after them, if its length is also known;
 * otherwise, the elements that cannot be placed are elements at unknown indices
 * of the new list (see {@link ListCreation#UNKNOWN_INDEX}), that surely has an
 * element if one of the operands surely has one (see
 * {@link ListModel#NONEMPTY}). {@code TypeError} is raised if {@code other}
 * might not be a list, and surely raised if it is surely not a list. This also
 * handles the receivers whose type is unknown, that might be lists: the result
 * is then an unknown value (see {@link ListOperations}).
 */
public class ListAdd extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected ListAdd(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static ListAdd build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListAdd(cfg, location, "__add__", exprs[0], exprs[1]);
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		Set<Type> self = analysis.getRuntimeTypesOf(state, left, st);
		if (self.isEmpty() || !self.stream().allMatch(ListOperations.LIST))
			// the receiver might not be a list (e.g. its type is unknown): the
			// result is not tracked
			return ListOperations.unknownResult(interprocedural, state, right, ListOperations.LIST, st);
		Set<Type> types = analysis.getRuntimeTypesOf(state, right, st);
		boolean unknown = types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st);
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		if (unknown || types.stream().anyMatch(ListOperations.LIST))
			result = concat(interprocedural, state, left, right, expressions);
		if (unknown || !types.stream().allMatch(ListOperations.LIST))
			raising.raise(state, st.getCFG(), loc, st, LibrarySpecificationProvider.TYPE_ERROR);
		return raising.moveFrom(result);
	}

	// the new list with the elements of left, and then the ones of right
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> concat(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		ListCreation creation = new ListCreation(getCFG(), new DerivedLocation(loc, "concat"));
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		// the operands are read after the allocation, that might have made
		// their allocation site weak (e.g. xs = xs + ys in a loop)
		Integer leftLength = ListModel.length(analysis, created, left, st);
		Integer rightLength = leftLength == null ? null : ListModel.length(analysis, created, right, st);
		boolean nonEmpty = nonEmpty(analysis, created, left) || nonEmpty(analysis, created, right);

		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression list : created.getExecutionExpressions()) {
			HeapDereference target = new HeapDereference(listType, list, loc);
			AnalysisState<A> filled = created;
			// the elements at known indices
			int index = 0;
			if (leftLength != null)
				for (int i = 0; i < leftLength; i++, index++)
					filled = copy(analysis, filled, created, left, i, target, index);
			if (rightLength != null)
				for (int i = 0; i < rightLength; i++, index++)
					filled = copy(analysis, filled, created, right, i, target, index);

			// the elements at unknown indices
			Set<SymbolicExpression> elements = new HashSet<>();
			if (leftLength == null)
				elements.addAll(elements(analysis, created, left));
			if (rightLength == null)
				elements.addAll(elements(analysis, created, right));
			if (!elements.isEmpty()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, target,
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				AnalysisState<A> unknowns = filled.bottomExecution();
				for (SymbolicExpression element : elements)
					unknowns = unknowns.lub(analysis.assign(filled, unknown, element, st));
				filled = unknowns;
				if (nonEmpty)
					filled = ListModel.setNonEmpty(analysis, filled, list, true, st);
			}
			result = result.lub(filled.withExecutionExpression(list));
		}
		return result;
	}

	// copies the element at index i of the list from into the element at
	// index index of the new list
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> copy(
			Analysis<A, D> analysis,
			AnalysisState<A> current,
			AnalysisState<A> operands,
			SymbolicExpression from,
			int i,
			HeapDereference target,
			int index)
			throws SemanticException {
		CodeLocation loc = getLocation();
		HeapDereference source = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.LIST), from,
				loc);
		AccessChild slot = new AccessChild(Untyped.INSTANCE, target, new Constant(Int32Type.INSTANCE, index, loc),
				loc);
		AnalysisState<A> result = current.bottomExecution();
		for (SymbolicExpression element : analysis.smallStepSemantics(operands,
				new AccessChild(Untyped.INSTANCE, source, new Constant(Int32Type.INSTANCE, i, loc), loc), st)
				.getExecutionExpressions())
			result = result.lub(analysis.assign(current, slot, element, st));
		return result;
	}

	// any of the elements of the list, or an unknown value if the operand
	// might not be a list whose elements are known
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<SymbolicExpression> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list)
			throws SemanticException {
		Set<SymbolicExpression> elements = new HashSet<>();
		Set<Type> types = analysis.getRuntimeTypesOf(state, list, st);
		if (!types.isEmpty() && types.stream().allMatch(ListModel::isList))
			elements.addAll(ListModel.elements(analysis, state, list, st).getExecutionExpressions().elements());
		else
			elements.add(new PushAny(Untyped.INSTANCE, getLocation()));
		return elements;
	}

	// whether the list surely has an element
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean nonEmpty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression list)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, list, st);
		return !types.isEmpty() && types.stream().allMatch(ListModel::isList)
				&& (ListModel.shape(analysis, state, list, st).prefix() > 0
						|| ListModel.nonEmpty(analysis, state, list, st));
	}
}
