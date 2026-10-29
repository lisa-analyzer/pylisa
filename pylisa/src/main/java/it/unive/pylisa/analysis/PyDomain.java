package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.heap.HeapLattice;
import it.unive.lisa.analysis.type.TypeDomain;
import it.unive.lisa.analysis.type.TypeLattice;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.analysis.value.ValueLattice;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * The {@link SimpleAbstractDomain} of python programs, where a state with a
 * bottom component is the bottom state. In {@link SimpleAbstractDomain}, a
 * state is bottom only if all its components are: assuming a condition that
 * cannot hold (e.g. the one of an assertion that surely fails) yields a bottom
 * value and type state, but the heap state is not bottom, and the state would
 * still be analyzed as a reachable one, where the variables are not bound
 * anymore. A state with a bottom component denotes no concrete state: here, it
 * is the bottom state.<br>
 * <br>
 * When a list is assumed to be truthy (e.g. in {@code if xs:}), it is also
 * recorded that it surely has an element (see {@code ListModel#NONEMPTY}).
 *
 * @param <H> the type of lattice of the heap domain
 * @param <V> the type of lattice of the value domain
 * @param <T> the type of lattice of the type domain
 */
public class PyDomain<H extends HeapLattice<H>, V extends ValueLattice<V>, T extends TypeLattice<T>>
		extends
		SimpleAbstractDomain<H, V, T> {

	/**
	 * Builds the domain.
	 *
	 * @param heapDomain  the heap domain
	 * @param valueDomain the value domain
	 * @param typeDomain  the type domain
	 */
	public PyDomain(
			HeapDomain<H> heapDomain,
			ValueDomain<V> valueDomain,
			TypeDomain<T> typeDomain) {
		super(heapDomain, valueDomain, typeDomain);
	}

	@Override
	public SimpleAbstractState<H, V, T> assign(
			SimpleAbstractState<H, V, T> state,
			Identifier id,
			SymbolicExpression expression,
			ScopeId<?> context,
			ProgramPoint pp)
			throws SemanticException {
		return normalize(super.assign(state, id, expression, context, pp));
	}

	@Override
	public SimpleAbstractState<H, V, T> smallStepSemantics(
			SimpleAbstractState<H, V, T> state,
			SymbolicExpression expression,
			ScopeId<?> context,
			ProgramPoint pp)
			throws SemanticException {
		return normalize(super.smallStepSemantics(state, expression, context, pp));
	}

	@Override
	public SimpleAbstractState<H, V, T> assume(
			SimpleAbstractState<H, V, T> state,
			SymbolicExpression expression,
			ScopeId<?> context,
			ProgramPoint src,
			ProgramPoint dest)
			throws SemanticException {
		return truthy(normalize(super.assume(state, expression, context, src, dest)), expression, context, src);
	}

	@Override
	public SimpleAbstractState<H, V, T> onCallReturn(
			SimpleAbstractState<H, V, T> entryState,
			SimpleAbstractState<H, V, T> callres,
			ScopeId<?> context,
			ProgramPoint call)
			throws SemanticException {
		return normalize(super.onCallReturn(entryState, callres, context, call));
	}

	// the bottom state, if any component of the given one is bottom
	private static <H extends HeapLattice<H>,
			V extends ValueLattice<V>,
			T extends TypeLattice<T>> SimpleAbstractState<H, V, T> normalize(
					SimpleAbstractState<H, V, T> state) {
		if (!state.isBottom()
				&& (state.heapState.isBottom() || state.valueState.isBottom() || state.typeState.isBottom()))
			return state.bottom();
		return state;
	}

	// after assuming that a list is truthy (e.g. xs in if xs:, while xs:, or
	// xs and ...), it surely has an element (see ListModel#NONEMPTY): this
	// holds for a variable pointing to a single list, that is not the summary
	// of many
	private SimpleAbstractState<H, V, T> truthy(
			SimpleAbstractState<H, V, T> state,
			SymbolicExpression expression,
			ScopeId<?> context,
			ProgramPoint pp)
			throws SemanticException {
		if (state.isBottom() || !(expression instanceof Identifier))
			return state;
		Set<Type> types = makeOracle(state, context).getRuntimeTypesOf(expression, pp);
		if (types.isEmpty() || !types.stream().allMatch(ListModel::isList))
			return state;
		HeapEnvWithFields heap = state.getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || !heap.knowsIdentifier((Identifier) expression))
			return state;
		AllocationSites sites = heap.getState((Identifier) expression);
		if (sites.isTop() || sites.isBottom() || sites.elements().size() != 1)
			return state;
		AllocationSite site = sites.elements().iterator().next();
		if (site.isWeak())
			return state;
		CodeLocation loc = pp.getLocation();
		Identifier child = site.withField(new Variable(Untyped.INSTANCE, ListModel.NONEMPTY, loc));
		return assign(state, child, new Constant(BoolType.INSTANCE, true, loc), context, pp);
	}
}
