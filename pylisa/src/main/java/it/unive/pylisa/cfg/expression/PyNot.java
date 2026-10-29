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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, expr, context, this);
		boolean bools = false, values = types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this),
				truthy = false, unknown = false;
		if (!values)
			for (Type t : types)
				if (t.isBooleanType())
					bools = true;
				else if (alwaysTrue(t))
					truthy = true;
				else if (t.isPointerType())
					// e.g. an object whose class defines __bool__
					unknown = true;
				else
					values = true;

		AnalysisState<A> result = state.bottomExecution();
		if (bools)
			result = result.lub(analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, expr,
							LogicalNegation.INSTANCE, loc),
					context, this));
		if (values)
			result = result.lub(analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, expr, LogicalNot.INSTANCE,
							loc),
					context, this));
		if (truthy)
			result = result.lub(
					analysis.smallStepSemantics(state, new Constant(BoolType.INSTANCE, false, loc), context, this));
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(BoolType.INSTANCE, loc), context, this));
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
