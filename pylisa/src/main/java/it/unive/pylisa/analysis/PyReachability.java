package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Reachability;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ReachLattice.ReachabilityStatus;
import it.unive.lisa.lattices.ReachabilityProduct;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Expression;
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
 * The {@link Reachability} of python programs, where an unreachable state is
 * bottom. In {@link Reachability}, the state of a branch that is not taken
 * (e.g. the {@code else} of {@code if x == 5}, with {@code x} surely {@code 5})
 * is unreachable, but it is not bottom: its reachability still maps the guards
 * traversed to reach it, so that the product with the bottom state of the
 * underlying domain is not bottom. The semantics of pylisa instead detects
 * unreachable code through bottom states (e.g. an assignment in unreachable
 * code assigns nothing, as its target has no type): here, a state whose
 * underlying state is bottom, or whose reachability is
 * {@link ReachabilityStatus#UNREACHABLE}, is the bottom state. No reachability
 * information is lost: the guards are restored at the end of their structures
 * (see {@link Reachability}) from the states reaching them, and an unreachable
 * state does not reach them.<br>
 * <br>
 * When a list is assumed to be truthy (e.g. in {@code if xs:}), it is also
 * recorded that it surely has an element (see {@code ListModel#NONEMPTY}).
 *
 * @param <D> the type of the underlying domain
 * @param <A> the type of lattice tracked by the underlying domain
 */
public class PyReachability<D extends AbstractDomain<A>, A extends AbstractLattice<A>> extends Reachability<D, A> {

	/**
	 * Builds the reachability domain.
	 *
	 * @param domain the underlying domain
	 */
	public PyReachability(
			D domain) {
		super(domain);
	}

	@Override
	public ReachabilityProduct<A> assign(
			ReachabilityProduct<A> state,
			Identifier id,
			SymbolicExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		return normalize(super.assign(state, id, expression, pp));
	}

	@Override
	public ReachabilityProduct<A> smallStepSemantics(
			ReachabilityProduct<A> state,
			SymbolicExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		return normalize(super.smallStepSemantics(state, expression, pp));
	}

	@Override
	public ReachabilityProduct<A> assume(
			ReachabilityProduct<A> state,
			SymbolicExpression expression,
			ProgramPoint src,
			ProgramPoint dest)
			throws SemanticException {
		if (src == dest || (src instanceof Expression && ((Expression) src).getParentStatement() != null))
			// an assumption made inside the semantics of a statement (e.g.
			// the short-circuit of and, or the branches of a conditional
			// expression), not when traversing an edge of the cfg: the
			// reachability is the one of the statement, as Reachability
			// tracks it only for the guards of the cfg
			return truthy(normalize(
					new ReachabilityProduct<>(state.first, domain.assume(state.second, expression, src, dest))),
					expression, src);
		return truthy(normalize(super.assume(state, expression, src, dest)), expression, src);
	}

	// after assuming that a list is truthy (e.g. xs in if xs:, while xs:, or
	// xs and ...), it surely has an element (see ListModel#NONEMPTY): this
	// holds for a variable pointing to a single list, that is not the summary
	// of many
	private ReachabilityProduct<A> truthy(
			ReachabilityProduct<A> state,
			SymbolicExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		if (state.isBottom() || !(expression instanceof Identifier))
			return state;
		Set<Type> types = domain.makeOracle(state.second).getRuntimeTypesOf(expression, pp);
		if (types.isEmpty() || !types.stream().allMatch(ListModel::isList))
			return state;
		HeapEnvWithFields heap = state.second.getLatticeInstance(HeapEnvWithFields.class);
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
		A refined = domain.assign(state.second, child, new Constant(BoolType.INSTANCE, true, loc), pp);
		return normalize(new ReachabilityProduct<>(state.first, refined));
	}

	@Override
	public ReachabilityProduct<A> onCallReturn(
			ReachabilityProduct<A> entryState,
			ReachabilityProduct<A> callres,
			ProgramPoint call)
			throws SemanticException {
		return normalize(super.onCallReturn(entryState, callres, call));
	}

	// the bottom state, if the given one is unreachable
	private static <A extends AbstractLattice<A>> ReachabilityProduct<A> normalize(
			ReachabilityProduct<A> state) {
		if (!state.isBottom() && (state.second.isBottom() || state.first.lattice == ReachabilityStatus.UNREACHABLE))
			return state.bottom();
		return state;
	}
}
