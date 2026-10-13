package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
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
		CodeLocation loc = getLocation();
		Set<Type> types = analysis.getRuntimeTypesOf(state, expr, this);
		boolean bools = false, values = types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this),
				truthy = false, unknown = false;
		if (!values)
			for (Type t : types)
				if (t.isBooleanType())
					bools = true;
				else if (t.isCodeMemberType() || alwaysTrue(t))
					truthy = true;
				else if (t.isPointerType())
					// e.g. an object whose class defines __bool__
					unknown = true;
				else
					values = true;

		AnalysisState<A> result = state.bottom();
		if (bools)
			result = result.lub(analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, expr,
							LogicalNegation.INSTANCE, loc),
					this));
		if (values)
			result = result.lub(analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.UnaryExpression(BoolType.INSTANCE, expr, LogicalNot.INSTANCE,
							loc),
					this));
		if (truthy)
			result = result.lub(analysis.smallStepSemantics(state, new Constant(BoolType.INSTANCE, false, loc), this));
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(BoolType.INSTANCE, loc), this));
		return result;
	}

	// whether the objects of type t are always true: this holds for objects of
	// classes whose methods are all known, and that define neither __bool__
	// nor __len__
	private static boolean alwaysTrue(
			Type t) {
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType() || !PyBinaryDispatch.fullyKnown(t))
			return false;
		CompilationUnit unit = t.asPointerType().getInnerType().asUnitType().getUnit();
		return unit.getInstanceCodeMembersByName("__bool__", true).isEmpty()
				&& unit.getInstanceCodeMembersByName("__len__", true).isEmpty();
	}
}
