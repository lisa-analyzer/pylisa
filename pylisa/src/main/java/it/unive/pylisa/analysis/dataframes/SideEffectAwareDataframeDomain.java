package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.symbolic.operators.dataframes.DataframeOperatorWithSideEffects;

public class SideEffectAwareDataframeDomain implements ValueDomain<SideEffectAwareDataframeDomain> {

	private final ValueEnvironment<DFOrConstant> env;

	public SideEffectAwareDataframeDomain() {
		this(new ValueEnvironment<>(new DFOrConstant()));
	}

	public SideEffectAwareDataframeDomain(ValueEnvironment<DFOrConstant> env) {
		this.env = env;
	}

	public ValueEnvironment<DFOrConstant> getEnv() {
		return env;
	}

	@Override
	public SideEffectAwareDataframeDomain assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		if (!env.getKeys().contains(id) && id instanceof AllocationSite) {
			// TODO this is very fragile and only works with the current state
			// of the field sensitive program point based heap
			AllocationSite as = (AllocationSite) id;
			if (as.getName().endsWith("]")) 
				id = new AllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(), as.getCodeLocation());
		}
		
		return new SideEffectAwareDataframeDomain(env.assign(id, expression, pp));
	}

	@Override
	public SideEffectAwareDataframeDomain smallStepSemantics(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		ValueEnvironment<DFOrConstant> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain(sss);
		return new SideEffectAwareDataframeDomain(env.smallStepSemantics(expression, pp));
	}

	private ValueEnvironment<DFOrConstant> sideEffect(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {

		ValueExpression dfVar = null;
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() instanceof DataframeOperatorWithSideEffects)
				dfVar = (ValueExpression) ((DataframeOperatorWithSideEffects) unary.getOperator()).getDataFrame(unary);
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			if (binary.getOperator() instanceof DataframeOperatorWithSideEffects)
				dfVar = (ValueExpression) ((DataframeOperatorWithSideEffects) binary.getOperator())
						.getDataFrame(binary);
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			if (ternary.getOperator() instanceof DataframeOperatorWithSideEffects)
				dfVar = (ValueExpression) ((DataframeOperatorWithSideEffects) ternary.getOperator())
						.getDataFrame(ternary);
		}

		ValueEnvironment<DFOrConstant> sss = env.bottom();
		if (dfVar == null || !(dfVar instanceof Identifier))
			return sss;

		if (!env.getKeys().contains(dfVar) && dfVar instanceof AllocationSite) {
			// TODO this is very fragile and only works with the current state
			// of the field sensitive program point based heap
			AllocationSite as = (AllocationSite) dfVar;
			if (as.getName().endsWith("]"))
				dfVar = new AllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(), as.getCodeLocation());
			else
				return sss;
			
			if (!env.getKeys().contains(dfVar))
				return sss;
		}

		sss = env.smallStepSemantics(expression, pp);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DFOrConstant updated = new DFOrConstant(stack);
		return sss.putState((Identifier) dfVar, updated);
	}

	@Override
	public SideEffectAwareDataframeDomain assume(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		ValueEnvironment<DFOrConstant> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain(sss.assume(expression, pp));
		return new SideEffectAwareDataframeDomain(env.assume(expression, pp));
	}

	@Override
	public SideEffectAwareDataframeDomain forgetIdentifier(Identifier id) throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.forgetIdentifier(id));
	}

	@Override
	public Satisfiability satisfies(ValueExpression expression, ProgramPoint pp) throws SemanticException {
		return env.satisfies(expression, pp);
	}

	@Override
	public SideEffectAwareDataframeDomain pushScope(ScopeToken token) throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.pushScope(token));
	}

	@Override
	public SideEffectAwareDataframeDomain popScope(ScopeToken token) throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.popScope(token));
	}

	@Override
	public DomainRepresentation representation() {
		return env.representation();
	}

	@Override
	public SideEffectAwareDataframeDomain lub(SideEffectAwareDataframeDomain other)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.lub(other.env));
	}

	@Override
	public SideEffectAwareDataframeDomain widening(SideEffectAwareDataframeDomain other)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.widening(other.env));
	}

	@Override
	public boolean lessOrEqual(SideEffectAwareDataframeDomain other) throws SemanticException {
		return env.lessOrEqual(other.env);
	}

	@Override
	public SideEffectAwareDataframeDomain top() {
		return new SideEffectAwareDataframeDomain(env.top());
	}

	@Override
	public boolean isTop() {
		return env.isTop();
	}

	@Override
	public SideEffectAwareDataframeDomain bottom() {
		return new SideEffectAwareDataframeDomain(env.bottom());
	}

	@Override
	public boolean isBottom() {
		return env.isBottom();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((env == null) ? 0 : env.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		SideEffectAwareDataframeDomain other = (SideEffectAwareDataframeDomain) obj;
		if (env == null) {
			if (other.env != null)
				return false;
		} else if (!env.equals(other.env))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return representation().toString();
	}
}
