package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Predicate;

/**
 * The shared parts of the operations on lists that build a new list
 * (concatenation, see {@link ListAdd}, and repetition, see {@link ListMul}).
 */
final class ListOperations {

	/**
	 * A list.
	 */
	static final Predicate<Type> LIST = t -> t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
			&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
					.equals(LibrarySpecificationProvider.LIST);

	/**
	 * An {@code int} (or a {@code bool}).
	 */
	static final Predicate<Type> INT = t -> (t.isNumericType() && t.asNumericType().isIntegral())
			|| t.isBooleanType();

	private ListOperations() {
	}

	/**
	 * Yields the location where the list built by an operation is allocated.
	 * Nested operations start at the same location (e.g. the two additions of
	 * {@code a + b + c}), so that the location derives from the one of the
	 * right operand, that differs.
	 *
	 * @param st  the statement performing the operation
	 * @param tag the tag of the operation
	 *
	 * @return the location of the allocation
	 */
	static CodeLocation allocation(
			Statement st,
			String tag) {
		// the operation is a binary expression, or the call of the method
		// implementing it (e.g. __add__(a, b), see PyBinaryDispatch)
		CodeLocation base = st.getLocation();
		if (st instanceof NaryExpression && ((NaryExpression) st).getSubExpressions().length > 1) {
			Expression[] operands = ((NaryExpression) st).getSubExpressions();
			base = operands[operands.length - 1].getLocation();
		}
		return new DerivedLocation(base, tag);
	}

	/**
	 * The semantics of an operation on a list whose result is not tracked.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the operands have been evaluated
	 * @param other           the operand that is not the list
	 * @param accepted        the types accepted for {@code other}
	 * @param st              the statement performing the operation
	 *
	 * @return the state after the operation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknownResult(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression other,
			Predicate<Type> accepted,
			Statement st)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		AnalysisState<A> result = analysis.smallStepSemantics(state,
				new PushAny(Untyped.INSTANCE, st.getLocation()), context, st);
		Set<Type> types = analysis.getRuntimeTypesOf(state, other, context, st);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st) || !types.stream().allMatch(accepted)) {
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
			raising.raise(state, st.getCFG(), st.getLocation(), st, LibrarySpecificationProvider.TYPE_ERROR);
			return raising.moveFrom(result);
		}
		return result;
	}

	// copies the element at index i of the list from, read in operands, into
	// the element at index index of the new list target
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> copy(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> current,
			AnalysisState<A> operands,
			SymbolicExpression from,
			int i,
			HeapDereference target,
			int index,
			Statement st)
			throws SemanticException {
		CodeLocation loc = st.getLocation();
		HeapDereference source = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.LIST), from,
				loc);
		AccessChild slot = new AccessChild(Untyped.INSTANCE, target, new Constant(Int32Type.INSTANCE, index, loc),
				loc);
		AnalysisState<A> result = current.bottomExecution();
		for (SymbolicExpression element : analysis.smallStepSemantics(operands,
				new AccessChild(Untyped.INSTANCE, source, new Constant(Int32Type.INSTANCE, i, loc), loc), context, st)
				.getExecutionExpressions())
			result = result.lub(analysis.assign(current, slot, element, context, st));
		return result;
	}

	// any of the elements of the list, or an unknown value if the operand
	// might not be a list whose elements are known
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<SymbolicExpression> elements(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression list,
			Statement st)
			throws SemanticException {
		Set<SymbolicExpression> elements = new HashSet<>();
		Set<Type> types = analysis.getRuntimeTypesOf(state, list, context, st);
		if (!types.isEmpty() && types.stream().allMatch(ListModel::isList))
			elements.addAll(
					ListModel.elements(analysis, context, state, list, st).getExecutionExpressions().elements());
		else
			elements.add(new PushAny(Untyped.INSTANCE, st.getLocation()));
		return elements;
	}

	// whether the list surely has an element
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean nonEmpty(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression list,
			Statement st)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, list, context, st);
		return !types.isEmpty() && types.stream().allMatch(ListModel::isList)
				&& (ListModel.shape(analysis, context, state, list, st).prefix() > 0
						|| ListModel.nonEmpty(analysis, context, state, list, st));
	}
}
