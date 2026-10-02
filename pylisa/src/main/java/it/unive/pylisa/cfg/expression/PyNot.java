package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.Untyped;

/**
 * Python's logical-negation operator (`not X`). Distinct from lisa-sdk's
 * {@link it.unive.lisa.program.cfg.statement.logic.Not}, which models
 * Java/C-style `!x` and requires the operand to have a {@code BooleanType}
 * — collapsing execution to {@code bottomExecution()} when it does not.
 * <p>
 * In Python, {@code not} is defined for every value: it returns
 * {@code False} for any truthy value and {@code True} for any falsy value
 * ({@code None}, {@code 0}, {@code ""}, {@code []}, {@code {{}}}, empty
 * iterables, …). The runtime type of the operand is therefore irrelevant
 * — we always emit a {@link LogicalNegation} symbolic expression without
 * any guard on the operand's runtime types. This avoids the bottom
 * cascade observed on IBM/mcp-context-forge's
 * {@code if not logging.getLogger().handlers:} pattern, where the
 * operand reads through the heap fallback's synthesized allocation site
 * and carries no {@code BooleanType} in its runtime-type set.
 * <p>
 * The static type stays {@code Untyped} (matches the symbolic value of
 * Python's `not`, which the analyzer can refine to bool elsewhere if
 * the value lattice supports it). The downstream small-step semantics
 * of {@link LogicalNegation} on a {@code ⊤} operand returns {@code ⊤}
 * in the value lattice (see {@code BaseNonRelationalDomain.unknownValue}
 * and {@code ConstantPropagation.evalUnaryExpression}), so the abstract
 * state stays non-{@code ⊥} for any caller-visible variable.
 */
public class PyNot extends UnaryExpression {

	public PyNot(
			CFG cfg,
			CodeLocation loc,
			Expression expression) {
		super(cfg, loc, "not", Untyped.INSTANCE, expression);
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
		return interprocedural.getAnalysis().smallStepSemantics(
				state,
				new it.unive.lisa.symbolic.value.UnaryExpression(
						Untyped.INSTANCE,
						expr,
						LogicalNegation.INSTANCE,
						getLocation()),
				this);
	}
}
