package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import java.util.Set;

/**
 * Python's {@code not x}, that is defined for values of any type: it is
 * {@code True} if {@code x} is false (e.g. {@code 0}, {@code ""} or
 * {@code None}, see {@link LogicalNot}), {@code False} otherwise. Functions,
 * and objects whose class defines neither {@code __bool__} nor {@code __len__},
 * are always true.
 */
public class PyNot extends UnaryExpression {

	/**
	 * Builds the expression.
	 *
	 * @param cfg      the cfg where the expression is
	 * @param location the location of the expression
	 * @param operand  the negated operand
	 */
	public PyNot(
			CFG cfg,
			CodeLocation location,
			Expression operand) {
		super(cfg, location, "!", BoolType.INSTANCE, operand);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return truthValue(interprocedural.getAnalysis(), interprocedural.getCurrentContext(), state, expr, true,
				this);
	}

	/**
	 * Computes Python's truth value of {@code value} ({@code bool(value)}), or
	 * its negation ({@code not value}), for each type of the value, in the
	 * executions where it has that type (see {@link PyBinaryDispatch#ofType}):
	 * bools are themselves, values that are always true (see
	 * {@link #alwaysTrue(Type)}) are true, objects whose class might define
	 * {@code __bool__} or {@code __len__} have an unknown truth value, and the
	 * other values (e.g. lists, numbers and strings) have a symbolic one (so
	 * that, e.g., assuming that {@code not xs} does not hold refines the list
	 * {@code xs}, see {@link it.unive.pylisa.analysis.PyDomain}).
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the state where the value has been computed
	 * @param value    the value
	 * @param negated  whether the negation of the truth value is computed
	 * @param pp       the program point computing the truth value
	 *
	 * @return the state computing the truth value
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> truthValue(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression value,
			boolean negated,
			Statement pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, context, pp);
		it.unive.lisa.symbolic.value.UnaryExpression not = new it.unive.lisa.symbolic.value.UnaryExpression(
				BoolType.INSTANCE, value, LogicalNot.INSTANCE, loc);
		// the symbolic truth value: not x, or not not x
		SymbolicExpression symbolic = negated ? not
				: new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, not,
						LogicalNegation.INSTANCE, loc);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp))
			return analysis.smallStepSemantics(state, symbolic, context, pp);

		AnalysisState<A> result = state.bottomExecution();
		for (Type t : types) {
			AnalysisState<A> typed = PyBinaryDispatch.ofType(analysis, context, state, value, t, types, pp);
			if (typed.getExecutionState().isBottom())
				continue;
			SymbolicExpression truth;
			if (t.isBooleanType())
				truth = negated ? new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, value,
						LogicalNegation.INSTANCE, loc) : value;
			else if (alwaysTrue(t))
				truth = new Constant(BoolType.INSTANCE, !negated, loc);
			else if (ListModel.isList(t) || !t.isPointerType())
				truth = symbolic;
			else
				// e.g. an object whose class defines __bool__
				truth = new PushAny(BoolType.INSTANCE, loc);
			result = result.lub(analysis.smallStepSemantics(typed, truth, context, pp));
		}
		return result;
	}

	/**
	 * Python's truth value of {@code value} (e.g. in {@code if value:}, see
	 * {@link #alwaysTrue(Type)}): it is not known for objects whose class
	 * defines {@code __bool__} or {@code __len__} (e.g. a list).
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param value    the value
	 * @param pp       the program point using the truth value
	 *
	 * @return whether the value is true
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability truth(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression value,
			Statement pp)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, context, pp);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp))
			return Satisfiability.UNKNOWN;
		if (types.stream().allMatch(PyNot::alwaysTrue))
			return Satisfiability.SATISFIED;
		if (types.stream().anyMatch(t -> t.isPointerType() || alwaysTrue(t)))
			return Satisfiability.UNKNOWN;
		Satisfiability sat = analysis.satisfies(state, value, context, pp);
		return sat == Satisfiability.BOTTOM ? Satisfiability.UNKNOWN : sat;
	}

	/**
	 * Whether the values of type {@code t} are always true: this holds for
	 * functions, and for objects of classes whose methods are all known, and
	 * that define neither {@code __bool__} nor {@code __len__}.
	 *
	 * @param t the type
	 *
	 * @return whether the values of the type are always true
	 */
	public static boolean alwaysTrue(
			Type t) {
		if (t.isCodeMemberType())
			return true;
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType() || !PyBinaryDispatch.fullyKnown(t))
			return false;
		CompilationUnit unit = t.asPointerType().getInnerType().asUnitType().getUnit();
		return unit.getInstanceCodeMembersByName("__bool__", true).isEmpty()
				&& unit.getInstanceCodeMembersByName("__len__", true).isEmpty();
	}
}
