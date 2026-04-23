package it.unive.pylisa.libraries.fastapi_cache;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.VariadicExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.statement.PassThroughLazyExpression;
import it.unive.pylisa.symbolic.operators.LazyEvaluatedExpressionOperator;
import java.util.HashMap;

/**
 * Pluggable statement for {@code fastapi_cache.decorator.cache(expire, ...)}.
 * Returns a {@link PassThroughLazyExpression} so that when the result is
 * applied to a function (i.e., used as a decorator), the function is passed
 * through unchanged. This allows the outer {@code @router.get(...)} decorator
 * to still receive the original handler and register the HTTP endpoint.
 */
public class CacheDecoratorFactory extends VariadicExpression implements PluggableStatement {

	protected Statement st;

	public CacheDecoratorFactory(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "fastapi_cache.cache", params, new HashMap<>() {
			{
				put("expire", 1);
				put("key_builder", 2);
				put("namespace", 3);
				put("coder", 4);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		CacheDecoratorFactory other = (CacheDecoratorFactory) o;
		int cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression[] combination,
			StatementStore<A> expressions)
			throws SemanticException {
		Expression[] callbacks = new Expression[] { new VariableRef(getCFG(), getLocation(), "callback") };
		PassThroughLazyExpression lazy = new PassThroughLazyExpression(
				Untyped.INSTANCE,
				new PushAny(Untyped.INSTANCE, getLocation()),
				LazyEvaluatedExpressionOperator.INSTANCE,
				getLocation(),
				callbacks);
		return interprocedural.getAnalysis().smallStepSemantics(state, lazy, this);
	}

	public static CacheDecoratorFactory build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CacheDecoratorFactory(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
