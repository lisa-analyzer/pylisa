package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class StarExpression extends UnaryExpression {

	private static final Logger LOG = LogManager.getLogger(StarExpression.class);

	public StarExpression(
			CFG cfg,
			CodeLocation loc,
			Expression expr) {
		super(cfg, loc, "*", expr);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	/**
	 * **Unsound translation** of {@code *expr} in call-argument and other
	 * value positions (e.g. {@code f(*args)}, {@code [*head, tail]} when
	 * lowered into a value context). Python semantics splice every element of
	 * the iterable into the surrounding argument list / collection; we don't
	 * model that. Treat {@code *xs} as {@code xs} — propagate the inner
	 * symbolic value as the result of this node so downstream call resolution
	 * still sees the iterable's identity (which is what routing/network
	 * analysis cares about). Element-wise unpack precision is lost; the WARN
	 * log makes the loss visible.
	 *
	 * <p>
	 * Pre-this-change we threw {@link it.unive.pylisa.UnsupportedStatementException}
	 * here, which aborted the whole analysis on every repo with a {@code *args}
	 * call site — ~9 repos in the bulk eval. Mirrors the skip-and-log applied
	 * to {@code star_expr} in tuple/list literal contexts at
	 * {@code LiteralVisitor.visitStarExprUnsound}.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		LOG.warn("[PyLiSA] Skipping star expression `*{}` at {} — analysis will "
				+ "treat `*xs` as `xs` (iterable-unpack semantics dropped, UNSOUND for "
				+ "callers that rely on element-wise argument fanout)",
				expr, getLocation());
		// Bind the inner's symbolic value as this node's computed expression.
		// Standard LiSA idiom for "this node has the value `expr`"; mirrors
		// other no-op pass-through expression classes in pylisa / lisa-network.
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}

}
