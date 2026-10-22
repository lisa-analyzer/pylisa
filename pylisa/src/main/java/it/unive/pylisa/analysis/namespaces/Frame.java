package it.unive.pylisa.analysis.namespaces;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.util.representation.ObjectRepresentation;
import it.unive.lisa.util.representation.SetRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * The binding status of the names of a namespace (e.g. the globals of a
 * module, or the locals of a function). The status of each name is one of the
 * {@link Binding}s, and it is represented through two sets: the names that are
 * definitely bound ({@code must}) and the ones that might be bound
 * ({@code may}, a superset of {@code must}). A name in {@code must} is
 * {@link Binding#BOUND}, a name in {@code may} but not in {@code must} is
 * {@link Binding#MAYBE}, and any other name is {@link Binding#UNBOUND}. After
 * executing code that might bind arbitrary names (e.g. {@code exec}), the
 * frame is <i>havocked</i>: every name might be bound, and no name is
 * definitely bound. Instances of this class are immutable.
 */
public class Frame implements BaseLattice<Frame> {

	/**
	 * The frame where no name is bound.
	 */
	public static final Frame EMPTY = new Frame(Collections.emptySet(), Collections.emptySet(), false);

	/**
	 * The top frame, where each name might be bound or not.
	 */
	public static final Frame TOP = new Frame(Collections.emptySet(), Collections.emptySet(), true);

	/**
	 * The bottom frame, used for unreachable code.
	 */
	public static final Frame BOTTOM = new Frame(Collections.emptySet(), Collections.emptySet(), false);

	private final Set<String> must;

	private final Set<String> may;

	private final boolean havoc;

	private Frame(
			Set<String> must,
			Set<String> may,
			boolean havoc) {
		this.must = must;
		// when havocked, every name is in may
		this.may = havoc ? Collections.emptySet() : may;
		this.havoc = havoc;
	}

	private static Frame mk(
			Set<String> must,
			Set<String> may,
			boolean havoc) {
		if (must.isEmpty() && havoc)
			return TOP;
		if (must.isEmpty() && may.isEmpty() && !havoc)
			return EMPTY;
		if (!havoc && !may.containsAll(must))
			// a name that is both definitely bound and definitely unbound:
			// the frame is unreachable
			return BOTTOM;
		return new Frame(Collections.unmodifiableSet(must), Collections.unmodifiableSet(may), havoc);
	}

	/**
	 * Yields the binding status of the given name.
	 *
	 * @param name the name
	 *
	 * @return the status of the name
	 */
	public Binding status(
			String name) {
		if (isBottom())
			return Binding.BOTTOM;
		if (must.contains(name))
			return Binding.BOUND;
		if (havoc || may.contains(name))
			return Binding.MAYBE;
		return Binding.UNBOUND;
	}

	/**
	 * Yields the names that are definitely bound in this frame.
	 *
	 * @return the names that are definitely bound
	 */
	public Set<String> getBound() {
		return must;
	}

	/**
	 * Yields the names that might be bound in this frame, including the ones
	 * that are definitely bound. If this frame is havocked (see
	 * {@link #isHavocked()}), any other name might be bound as well.
	 *
	 * @return the names that might be bound
	 */
	public Set<String> getMaybeBound() {
		return havoc ? must : may;
	}

	/**
	 * Yields whether this frame is havocked, that is, whether any name might
	 * be bound in it.
	 *
	 * @return whether this frame is havocked
	 */
	public boolean isHavocked() {
		return havoc;
	}

	/**
	 * Yields a copy of this frame where the given name is definitely bound.
	 *
	 * @param name the name
	 *
	 * @return the updated frame
	 */
	public Frame bind(
			String name) {
		if (isBottom() || must.contains(name))
			return this;
		Set<String> m = new HashSet<>(must);
		m.add(name);
		Set<String> p = new HashSet<>(may);
		p.add(name);
		return mk(m, p, havoc);
	}

	/**
	 * Yields a copy of this frame where the given name is not bound (e.g.
	 * after {@code del name}). If this frame is havocked, the name might still
	 * be bound.
	 *
	 * @param name the name
	 *
	 * @return the updated frame
	 */
	public Frame unbind(
			String name) {
		if (isBottom() || (!must.contains(name) && !may.contains(name)))
			return this;
		Set<String> m = new HashSet<>(must);
		m.remove(name);
		Set<String> p = new HashSet<>(may);
		p.remove(name);
		return mk(m, p, havoc);
	}

	/**
	 * Yields a copy of this frame where any name might be bound, and no name is
	 * definitely bound (e.g. after executing unknown code with {@code exec}).
	 *
	 * @return the havocked frame
	 */
	public Frame havoc() {
		return isBottom() ? this : TOP;
	}

	@Override
	public Frame lubAux(
			Frame other)
			throws SemanticException {
		Set<String> m = new HashSet<>(must);
		m.retainAll(other.must);
		Set<String> p = new HashSet<>(may);
		p.addAll(other.may);
		return mk(m, p, havoc || other.havoc);
	}

	@Override
	public Frame glbAux(
			Frame other)
			throws SemanticException {
		Set<String> m = new HashSet<>(must);
		m.addAll(other.must);
		Set<String> p;
		if (havoc && other.havoc)
			p = Collections.emptySet();
		else if (havoc)
			p = other.may;
		else if (other.havoc)
			p = may;
		else {
			p = new HashSet<>(may);
			p.retainAll(other.may);
		}
		return mk(m, p, havoc && other.havoc);
	}

	@Override
	public boolean lessOrEqualAux(
			Frame other)
			throws SemanticException {
		if (!must.containsAll(other.must))
			return false;
		if (other.havoc)
			return true;
		return !havoc && other.may.containsAll(may);
	}

	@Override
	public Frame top() {
		return TOP;
	}

	@Override
	public Frame bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isTop() {
		return this == TOP;
	}

	@Override
	public boolean isBottom() {
		return this == BOTTOM;
	}

	@Override
	public int hashCode() {
		return Objects.hash(must, may, havoc, isBottom());
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null || getClass() != obj.getClass())
			return false;
		Frame other = (Frame) obj;
		// BOTTOM and EMPTY have the same content
		return isBottom() == other.isBottom() && havoc == other.havoc && must.equals(other.must)
				&& may.equals(other.may);
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	@Override
	public StructuredRepresentation representation() {
		if (isBottom())
			return Lattice.bottomRepresentation();
		if (isTop())
			return Lattice.topRepresentation();

		Map<String, StructuredRepresentation> fields = new TreeMap<>();
		fields.put("bound", names(must));
		if (havoc)
			fields.put("maybe", new StringRepresentation("any"));
		else {
			Set<String> maybe = new HashSet<>(may);
			maybe.removeAll(must);
			fields.put("maybe", names(maybe));
		}
		return new ObjectRepresentation(fields);
	}

	private static StructuredRepresentation names(
			Set<String> names) {
		Set<StructuredRepresentation> elements = new TreeSet<>();
		for (String name : names)
			elements.add(new StringRepresentation(name));
		return new SetRepresentation(elements);
	}
}
