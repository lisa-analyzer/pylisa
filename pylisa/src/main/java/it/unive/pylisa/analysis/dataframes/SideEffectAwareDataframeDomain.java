package it.unive.pylisa.analysis.dataframes;

import java.util.HashSet;
import java.util.Map.Entry;
import java.util.Set;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.NonRelationalValueDomain;
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
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.DataframeOperatorWithSideEffects;
import it.unive.pylisa.symbolic.operators.SetOption;
import it.unive.pylisa.symbolic.operators.SetOptionAux;

public class SideEffectAwareDataframeDomain<T extends NonRelationalValueDomain<T> & DataframeAwareDomain<T, D>,
		D extends NonRelationalValueDomain<D>>
		implements ValueDomain<SideEffectAwareDataframeDomain<T, D>> {

	private final ValueEnvironment<T> env;

	public SideEffectAwareDataframeDomain(T singleton) {
		this(new ValueEnvironment<>(singleton));
	}

	private SideEffectAwareDataframeDomain(ValueEnvironment<T> env) {
		this.env = env;
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.assign(id, expression, pp));
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> smallStepSemantics(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			if (binary.getOperator() == SetOption.INSTANCE) {
				SideEffectAwareDataframeDomain<T, D> sss = bottom();
				for (Identifier key : allKeys()) {
					TernaryExpression aux = new TernaryExpression(PandasDataframeType.INSTANCE, key,
							binary.getLeft(), binary.getRight(), SetOptionAux.INSTANCE, pp.getLocation());
					sss = sss.lub(smallStepSemantics(aux, pp));
				}
				return sss;
			}
		}

		ValueEnvironment<T> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain<>(sss);
		return new SideEffectAwareDataframeDomain<>(env.smallStepSemantics(expression, pp));
	}

	private ValueEnvironment<T> sideEffect(ValueExpression expression, ProgramPoint pp)
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

		if (dfVar instanceof MemoryPointer)
			dfVar = ((MemoryPointer) dfVar).getReferencedLocation();
		ValueEnvironment<T> sss = env.bottom();
		if (dfVar == null || !env.getKeys().contains(dfVar))
			return sss;

		sss = env.smallStepSemantics(expression, pp);
		T singleton = sss.getValueOnStack();
		D stack = singleton.getDataFrame();
		T newValue = singleton.createDataframe(stack);

		ValueEnvironment<T> result = sss.bottom();
		D df = env.smallStepSemantics(dfVar, pp).getValueOnStack().getDataFrame();
		if (!df.isTop() && !df.isBottom())
			for (Identifier key : keysOf(df))
				result = result.lub(sss.putState(key, newValue));

		// this is to have the right value on the stack
		return result;
	}

	private Set<Identifier> keysOf(D df) {
		Set<Identifier> keys = new HashSet<>();
		for (Entry<Identifier, T> entry : env)
			if (entry.getValue().sameDataFrame(df))
				keys.add(entry.getKey());
		return keys;
	}

	private Set<Identifier> allKeys() {
		Set<Identifier> keys = new HashSet<>();
		for (Entry<Identifier, T> entry : env)
			if (entry.getKey().getRuntimeTypes().anyMatch(t -> t.equals(PandasDataframeType.INSTANCE)))
				keys.add(entry.getKey());
		return keys;
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> assume(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		ValueEnvironment<T> sss = sideEffect(expression, pp);
		if (!sss.isBottom())
			return new SideEffectAwareDataframeDomain<>(env.assume(expression, pp));
		return new SideEffectAwareDataframeDomain<>(env.assume(expression, pp));
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> forgetIdentifier(Identifier id) throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.forgetIdentifier(id));
	}

	@Override
	public Satisfiability satisfies(ValueExpression expression, ProgramPoint pp) throws SemanticException {
		return env.satisfies(expression, pp);
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> pushScope(ScopeToken token) throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.pushScope(token));
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> popScope(ScopeToken token) throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.popScope(token));
	}

	@Override
	public DomainRepresentation representation() {
		return env.representation();
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> lub(SideEffectAwareDataframeDomain<T, D> other)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.lub(other.env));
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> widening(SideEffectAwareDataframeDomain<T, D> other)
			throws SemanticException {
		return new SideEffectAwareDataframeDomain<>(env.widening(other.env));
	}

	@Override
	public boolean lessOrEqual(SideEffectAwareDataframeDomain<T, D> other) throws SemanticException {
		return env.lessOrEqual(other.env);
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> top() {
		return new SideEffectAwareDataframeDomain<>(env.top());
	}

	@Override
	public boolean isTop() {
		return env.isTop();
	}

	@Override
	public SideEffectAwareDataframeDomain<T, D> bottom() {
		return new SideEffectAwareDataframeDomain<>(env.bottom());
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
		SideEffectAwareDataframeDomain<?, ?> other = (SideEffectAwareDataframeDomain<?, ?>) obj;
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
