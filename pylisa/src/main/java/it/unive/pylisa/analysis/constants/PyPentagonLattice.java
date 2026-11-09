package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.combination.ValueCartesianCombination;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.lattices.symbolic.DefiniteIdSet;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.datastructures.trie.PatriciaTrieMap;
import it.unive.lisa.util.numeric.IntInterval;
import it.unive.lisa.util.representation.MapRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Predicate;

/**
 * The lattice of {@link PyPentagon}: the values of the variables (constants and
 * intervals, see {@link PyValue}) and, as in pentagons, the strict upper bounds
 * of the integer variables: {@code y} is a bound of {@code x} if {@code x < y}
 * surely holds. The lattice also tracks the copies of each variable (the
 * variables surely equal to it, e.g. after {@code n = len(xs)}), so that the
 * bounds of a variable are also the ones of its copies. The bounds and the
 * copies of a variable are {@link DefiniteIdSet}s, whose least upper bound is
 * the intersection (a relation holds after a join if it holds in both states),
 * and whose top element is the empty set (no relations). A bound also holds in
 * a state when it is implied by the intervals of the two variables (the upper
 * bound of {@code x} is less than the lower bound of {@code y}), so that it is
 * not lost when joining with a state where it was only known from them.
 */
public class PyPentagonLattice
		extends
		ValueCartesianCombination<PyPentagonLattice, ValueEnvironment<PyValue>, ValueEnvironment<DefiniteIdSet>> {

	/**
	 * The copies of the variables: the other variables surely equal to each of
	 * them.
	 */
	public final ValueEnvironment<DefiniteIdSet> copies;

	/**
	 * Builds the lattice, without copies.
	 *
	 * @param values the values of the variables
	 * @param bounds the strict upper bounds of the variables
	 */
	public PyPentagonLattice(
			ValueEnvironment<PyValue> values,
			ValueEnvironment<DefiniteIdSet> bounds) {
		this(values, bounds, new ValueEnvironment<>(new DefiniteIdSet(Collections.emptySet(), true)));
	}

	/**
	 * Builds the lattice.
	 *
	 * @param values the values of the variables
	 * @param bounds the strict upper bounds of the variables
	 * @param copies the copies of the variables
	 */
	public PyPentagonLattice(
			ValueEnvironment<PyValue> values,
			ValueEnvironment<DefiniteIdSet> bounds,
			ValueEnvironment<DefiniteIdSet> copies) {
		super(values, bounds);
		this.copies = copies;
	}

	/**
	 * Yields the environment with the given relations.
	 *
	 * @param lattice   the lattice of the relations
	 * @param relations the relations of the variables
	 *
	 * @return the environment
	 */
	static ValueEnvironment<DefiniteIdSet> environment(
			DefiniteIdSet lattice,
			Map<Identifier, Set<Identifier>> relations) {
		PatriciaTrieMap<Identifier, DefiniteIdSet> function = PatriciaTrieMap.empty();
		for (Map.Entry<Identifier, Set<Identifier>> entry : relations.entrySet())
			if (!entry.getValue().isEmpty())
				function = function.put(entry.getKey(), new DefiniteIdSet(new HashSet<>(entry.getValue())));
		return new ValueEnvironment<>(lattice, function);
	}

	@Override
	public PyPentagonLattice mk(
			ValueEnvironment<PyValue> values,
			ValueEnvironment<DefiniteIdSet> bounds) {
		// only used for the top and bottom elements
		return new PyPentagonLattice(values, bounds, copies.top());
	}

	/**
	 * Yields this lattice with the given values and bounds, and the same
	 * copies.
	 *
	 * @param values the values
	 * @param bounds the bounds
	 *
	 * @return the lattice
	 */
	public PyPentagonLattice with(
			ValueEnvironment<PyValue> values,
			ValueEnvironment<DefiniteIdSet> bounds) {
		return new PyPentagonLattice(values, bounds, copies);
	}

	// the relations of x in the environment
	private static Set<Identifier> of(
			ValueEnvironment<DefiniteIdSet> environment,
			Identifier x) {
		DefiniteIdSet set = environment.getState(x);
		return set.isTop() || set.isBottom() ? Set.of() : set.elements();
	}

	/**
	 * Yields the strict upper bounds of the variable.
	 *
	 * @param x the variable
	 *
	 * @return the variables surely greater than {@code x}
	 */
	public Set<Identifier> bounds(
			Identifier x) {
		return of(second, x);
	}

	/**
	 * Yields the variable and its copies.
	 *
	 * @param x the variable
	 *
	 * @return the variables surely equal to {@code x}, including it
	 */
	public Set<Identifier> equal(
			Identifier x) {
		Set<Identifier> result = new HashSet<>(of(copies, x));
		result.add(x);
		return result;
	}

	// the relations of all the variables, as a map
	private static Map<Identifier, Set<Identifier>> map(
			ValueEnvironment<DefiniteIdSet> environment) {
		Map<Identifier, Set<Identifier>> result = new HashMap<>();
		if (!environment.isBottom() && !environment.isTop())
			for (Identifier x : environment.getKeys())
				result.put(x, new HashSet<>(of(environment, x)));
		return result;
	}

	/**
	 * Yields the bounds of all the variables, as a map.
	 *
	 * @return the bounds
	 */
	Map<Identifier, Set<Identifier>> boundsMap() {
		return map(second);
	}

	/**
	 * Yields the copies of all the variables, as a map.
	 *
	 * @return the copies
	 */
	Map<Identifier, Set<Identifier>> copiesMap() {
		return map(copies);
	}

	// whether x < y surely holds, from the bounds (also of the copies) or from
	// the intervals
	boolean below(
			Identifier x,
			Identifier y) {
		Set<Identifier> greater = equal(y);
		for (Identifier xx : equal(x))
			for (Identifier yy : bounds(xx))
				if (greater.contains(yy))
					return true;
		return implied(first, x, y);
	}

	// whether x == y surely holds, from the copies or from the constants
	boolean same(
			Identifier x,
			Identifier y) {
		if (x.equals(y) || of(copies, x).contains(y))
			return true;
		IntInterval ix = first.getState(x).getInterval(), iy = first.getState(y).getInterval();
		return ix.isSingleton() && ix.equals(iy);
	}

	// whether the relation between x and y is vacuous in this state, as one of
	// them has no value (e.g. it is bound only on the other path of a join)
	private boolean absent(
			Identifier x,
			Identifier y) {
		return !first.knowsIdentifier(x) || !first.knowsIdentifier(y);
	}

	// whether the intervals of the values imply x < y
	private static boolean implied(
			ValueEnvironment<PyValue> values,
			Identifier x,
			Identifier y) {
		IntInterval ix = values.getState(x).getInterval(), iy = values.getState(y).getInterval();
		return !ix.isBottom() && !iy.isBottom() && ix.getHigh().compareTo(iy.getLow()) < 0;
	}

	@Override
	public boolean isBottom() {
		return first.isBottom();
	}

	@Override
	public boolean isTop() {
		// the top element has no copies: otherwise, it would be returned as
		// the least upper bound of states where they do not hold
		return first.isTop() && map(second).values().stream().allMatch(Set::isEmpty)
				&& map(copies).values().stream().allMatch(Set::isEmpty);
	}

	@Override
	public boolean lessOrEqualAux(
			PyPentagonLattice other)
			throws SemanticException {
		if (!first.lessOrEqual(other.first))
			return false;
		for (Identifier x : other.second.getKeys())
			for (Identifier y : other.bounds(x))
				if (!below(x, y) && !absent(x, y))
					return false;
		for (Identifier x : other.copies.getKeys())
			for (Identifier y : of(other.copies, x))
				if (!same(x, y) && !absent(x, y))
					return false;
		return true;
	}

	@Override
	public PyPentagonLattice lubAux(
			PyPentagonLattice other)
			throws SemanticException {
		// the relations holding in both states
		Map<Identifier, Set<Identifier>> bounds = new HashMap<>();
		Set<Identifier> keys = new HashSet<>(second.getKeys());
		keys.addAll(other.second.getKeys());
		for (Identifier x : keys) {
			Set<Identifier> common = new HashSet<>();
			for (Identifier y : bounds(x))
				if (other.below(x, y) || other.absent(x, y))
					common.add(y);
			for (Identifier y : other.bounds(x))
				if (below(x, y) || absent(x, y))
					common.add(y);
			bounds.put(x, common);
		}
		Map<Identifier, Set<Identifier>> equal = new HashMap<>();
		for (Identifier x : copies.getKeys()) {
			Set<Identifier> common = new HashSet<>();
			for (Identifier y : of(copies, x))
				if (other.same(x, y) || other.absent(x, y))
					common.add(y);
			equal.put(x, common);
		}
		for (Identifier x : other.copies.getKeys()) {
			Set<Identifier> common = equal.computeIfAbsent(x, k -> new HashSet<>());
			for (Identifier y : of(other.copies, x))
				if (same(x, y) || absent(x, y))
					common.add(y);
		}
		return new PyPentagonLattice(first.lub(other.first), environment(second.lattice, bounds),
				environment(copies.lattice, equal));
	}

	@Override
	public PyPentagonLattice upchainAux(
			PyPentagonLattice other)
			throws SemanticException {
		// the join of the fixpoint before widening: the relations holding in
		// both states, as for the least upper bound
		PyPentagonLattice lub = lubAux(other);
		return new PyPentagonLattice(first.upchain(other.first), lub.second, lub.copies);
	}

	@Override
	public PyPentagonLattice downchainAux(
			PyPentagonLattice other)
			throws SemanticException {
		PyPentagonLattice glb = glbAux(other);
		return new PyPentagonLattice(first.downchain(other.first), glb.second, glb.copies);
	}

	@Override
	public PyPentagonLattice wideningAux(
			PyPentagonLattice other)
			throws SemanticException {
		// the relations of this state that still hold: they can only decrease,
		// so that the widening converges
		Map<Identifier, Set<Identifier>> bounds = new HashMap<>();
		for (Identifier x : second.getKeys()) {
			Set<Identifier> kept = new HashSet<>();
			for (Identifier y : bounds(x))
				if (other.below(x, y) || other.absent(x, y))
					kept.add(y);
			bounds.put(x, kept);
		}
		Map<Identifier, Set<Identifier>> equal = new HashMap<>();
		for (Identifier x : copies.getKeys()) {
			Set<Identifier> kept = new HashSet<>();
			for (Identifier y : of(copies, x))
				if (of(other.copies, x).contains(y) || other.absent(x, y))
					kept.add(y);
			equal.put(x, kept);
		}
		return new PyPentagonLattice(first.widening(other.first), environment(second.lattice, bounds),
				environment(copies.lattice, equal));
	}

	@Override
	public PyPentagonLattice narrowingAux(
			PyPentagonLattice other)
			throws SemanticException {
		return new PyPentagonLattice(first.narrowing(other.first), second, copies);
	}

	@Override
	public PyPentagonLattice glbAux(
			PyPentagonLattice other)
			throws SemanticException {
		// the relations holding in any of the states
		Map<Identifier, Set<Identifier>> bounds = map(second);
		map(other.second).forEach((
				x,
				ys) -> bounds.computeIfAbsent(x, k -> new HashSet<>()).addAll(ys));
		Map<Identifier, Set<Identifier>> equal = map(copies);
		map(other.copies).forEach((
				x,
				ys) -> equal.computeIfAbsent(x, k -> new HashSet<>()).addAll(ys));
		return new PyPentagonLattice(first.glb(other.first), environment(second.lattice, bounds),
				environment(copies.lattice, equal));
	}

	/**
	 * Yields this lattice where the variables for which {@code removed} holds
	 * have no relations, and are not related to the other variables (their
	 * values are not changed).
	 *
	 * @param removed the variables to remove
	 *
	 * @return the lattice without the relations of the variables
	 */
	PyPentagonLattice without(
			Predicate<Identifier> removed) {
		Function<Identifier, Identifier> renaming = id -> removed.test(id) ? null : id;
		return new PyPentagonLattice(first, rename(second, renaming), rename(copies, renaming));
	}

	// the environment where each variable is renamed (or removed, if renamed
	// to null)
	private static ValueEnvironment<DefiniteIdSet> rename(
			ValueEnvironment<DefiniteIdSet> environment,
			Function<Identifier, Identifier> renaming) {
		if (environment.isBottom() || environment.isTop())
			return environment;
		Map<Identifier, Set<Identifier>> renamed = new HashMap<>();
		for (Identifier x : environment.getKeys()) {
			Identifier rx = renaming.apply(x);
			if (rx == null)
				continue;
			Set<Identifier> set = new HashSet<>();
			for (Identifier y : of(environment, x)) {
				Identifier ry = renaming.apply(y);
				if (ry != null)
					set.add(ry);
			}
			renamed.put(rx, set);
		}
		return environment(environment.lattice, renamed);
	}

	@Override
	public PyPentagonLattice forgetIdentifier(
			Identifier id,
			ProgramPoint pp)
			throws SemanticException {
		PyPentagonLattice relations = without(id::equals);
		return new PyPentagonLattice(first.forgetIdentifier(id, pp), relations.second, relations.copies);
	}

	@Override
	public PyPentagonLattice forgetIdentifiers(
			Iterable<Identifier> ids,
			ProgramPoint pp)
			throws SemanticException {
		Set<Identifier> removed = new HashSet<>();
		ids.forEach(removed::add);
		PyPentagonLattice relations = without(removed::contains);
		return new PyPentagonLattice(first.forgetIdentifiers(removed, pp), relations.second, relations.copies);
	}

	@Override
	public PyPentagonLattice forgetIdentifiersIf(
			Predicate<Identifier> test,
			ProgramPoint pp)
			throws SemanticException {
		PyPentagonLattice relations = without(test);
		return new PyPentagonLattice(first.forgetIdentifiersIf(test, pp), relations.second, relations.copies);
	}

	@Override
	public PyPentagonLattice pushScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		SemanticException[] error = new SemanticException[1];
		Function<Identifier, Identifier> renaming = id -> {
			try {
				return (Identifier) id.pushScope(token, pp);
			} catch (SemanticException e) {
				error[0] = e;
				return null;
			}
		};
		ValueEnvironment<DefiniteIdSet> bounds = rename(second, renaming), equal = rename(copies, renaming);
		if (error[0] != null)
			throw error[0];
		return new PyPentagonLattice(first.pushScope(token, pp), bounds, equal);
	}

	@Override
	public PyPentagonLattice popScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		SemanticException[] error = new SemanticException[1];
		Function<Identifier, Identifier> renaming = id -> {
			try {
				return (Identifier) id.popScope(token, pp);
			} catch (SemanticException e) {
				error[0] = e;
				return null;
			}
		};
		ValueEnvironment<DefiniteIdSet> bounds = rename(second, renaming), equal = rename(copies, renaming);
		if (error[0] != null)
			throw error[0];
		return new PyPentagonLattice(first.popScope(token, pp), bounds, equal);
	}

	@Override
	public PyPentagonLattice store(
			Identifier target,
			Identifier source)
			throws SemanticException {
		PyPentagonLattice cleaned = without(target::equals);
		// the summaries of many values (e.g. the fields of weak allocation
		// sites) have no relations
		PyPentagonLattice copied = PyValueDomain.single(target) && PyValueDomain.single(source)
				? cleaned.copy(target, source)
				: cleaned;
		return new PyPentagonLattice(first.store(target, source), copied.second, copied.copies);
	}

	/**
	 * Yields the relations where {@code target}, that has no relations, is a
	 * copy of {@code source}: it has the same bounds, it is a bound of the
	 * variables bounded by {@code source}, and it is equal to {@code source}
	 * and its copies (the values are not changed).
	 *
	 * @param target the copy
	 * @param source the copied variable
	 *
	 * @return the relations
	 */
	PyPentagonLattice copy(
			Identifier target,
			Identifier source) {
		Map<Identifier, Set<Identifier>> bounds = map(second);
		Set<Identifier> sources = equal(source);
		for (Set<Identifier> set : bounds.values())
			if (!Collections.disjoint(set, sources))
				set.add(target);
		bounds.put(target, new HashSet<>(bounds(source)));
		Map<Identifier, Set<Identifier>> equal = map(copies);
		for (Identifier s : sources)
			equal.computeIfAbsent(s, k -> new HashSet<>()).add(target);
		equal.put(target, sources);
		return new PyPentagonLattice(first, environment(second.lattice, bounds), environment(copies.lattice, equal));
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hashCode(copies);
	}

	@Override
	public boolean equals(
			Object obj) {
		return super.equals(obj) && Objects.equals(copies, ((PyPentagonLattice) obj).copies);
	}

	@Override
	public StructuredRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		boolean noBounds = second.isTop() || second.getKeys().isEmpty();
		boolean noCopies = copies.isTop() || copies.getKeys().isEmpty();
		if (noBounds && noCopies)
			return first.representation();
		Map<StructuredRepresentation, StructuredRepresentation> mapping = new HashMap<>();
		mapping.put(new StringRepresentation("values"), first.representation());
		if (!noBounds)
			mapping.put(new StringRepresentation("bounds"), relations(second));
		if (!noCopies)
			mapping.put(new StringRepresentation("copies"), relations(copies));
		return new MapRepresentation(mapping);
	}

	private static StructuredRepresentation relations(
			ValueEnvironment<DefiniteIdSet> environment) {
		Map<StructuredRepresentation, StructuredRepresentation> mapping = new HashMap<>();
		for (Identifier x : environment.getKeys())
			mapping.put(new StringRepresentation(x), environment.getState(x).representation());
		return new MapRepresentation(mapping);
	}
}
