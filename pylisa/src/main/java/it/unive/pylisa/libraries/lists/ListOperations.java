package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Set;
import java.util.function.Predicate;

/**
 * The operations on lists that build a new list (concatenation and repetition),
 * whose result is not tracked (see {@link ListAdd}): the result is an unknown
 * value, and {@code TypeError} is raised if the other operand might not be of
 * the accepted type.
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
		AnalysisState<A> result = analysis.smallStepSemantics(state,
				new PushAny(Untyped.INSTANCE, st.getLocation()), st);
		Set<Type> types = analysis.getRuntimeTypesOf(state, other, st);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st) || !types.stream().allMatch(accepted))
			result = result.lub(PyExceptions.raise(analysis, state, st.getCFG(), st.getLocation(), st,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
