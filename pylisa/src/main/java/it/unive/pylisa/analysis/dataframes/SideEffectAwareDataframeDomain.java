package it.unive.pylisa.analysis.dataframes;

import java.util.HashSet;
import java.util.Map.Entry;
import java.util.Set;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.symbolic.SideEffectOperator;

public class SideEffectAwareDataframeDomain implements ValueDomain<SideEffectAwareDataframeDomain> {

	private final ValueEnvironment<DataframeDomain> env;

	public SideEffectAwareDataframeDomain() {
		this(new ValueEnvironment<>(new DataframeDomain()).top());
	}

	private SideEffectAwareDataframeDomain(ValueEnvironment<DataframeDomain> env) {
		this.env = env;
	}

	@Override
	public SideEffectAwareDataframeDomain assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.assign(id, expression, pp));
	}

	@Override
	public SideEffectAwareDataframeDomain smallStepSemantics(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		ValueEnvironment<DataframeDomain> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain(sss);
		return new SideEffectAwareDataframeDomain(env.smallStepSemantics(expression, pp));
	}

	private ValueEnvironment<DataframeDomain> sideEffect(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {

		ValueExpression dfVar = null;
		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() instanceof SideEffectOperator)
				dfVar = (ValueExpression) ((SideEffectOperator) unary.getOperator()).getDataFrame(unary);
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			if (binary.getOperator() instanceof SideEffectOperator)
				dfVar = (ValueExpression) ((SideEffectOperator) binary.getOperator()).getDataFrame(binary);
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			if (ternary.getOperator() instanceof SideEffectOperator)
				dfVar = (ValueExpression) ((SideEffectOperator) ternary.getOperator()).getDataFrame(ternary);
		}

		ValueEnvironment<DataframeDomain> sss = env.bottom();
		if (dfVar == null)
			return sss;

		if (dfVar instanceof MemoryPointer)
			dfVar = ((MemoryPointer) dfVar).getReferencedLocation();
		DataframeTransformationDomain df = env.smallStepSemantics(dfVar, pp).getValueOnStack().left;

		if (!df.isTop() && !df.isBottom())
			for (Identifier key : keysOf(df))
				sss = sss.lub(env.assign(key, expression, pp));

		return sss;
	}

	private Set<Identifier> keysOf(DataframeTransformationDomain df) {
		Set<Identifier> keys = new HashSet<>();
		for (Entry<Identifier, DataframeDomain> entry : env)
			if (entry.getValue().left.equals(df))
				keys.add(entry.getKey());
		return keys;
	}

	@Override
	public SideEffectAwareDataframeDomain assume(ValueExpression expression, ProgramPoint pp) throws SemanticException {
		ValueEnvironment<DataframeDomain> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain(env.assume(expression, pp));
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
	public SideEffectAwareDataframeDomain lub(SideEffectAwareDataframeDomain other) throws SemanticException {
		return new SideEffectAwareDataframeDomain(env.lub(other.env));
	}

	@Override
	public SideEffectAwareDataframeDomain widening(SideEffectAwareDataframeDomain other) throws SemanticException {
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
