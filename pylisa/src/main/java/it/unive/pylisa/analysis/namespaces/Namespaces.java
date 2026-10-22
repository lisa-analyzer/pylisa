package it.unive.pylisa.analysis.namespaces;

import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.FixpointInfo;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.ScopedObject;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.util.representation.MapRepresentation;
import it.unive.lisa.util.representation.ObjectRepresentation;
import it.unive.lisa.util.representation.SetRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * The namespaces of a Python program during the analysis, that is, the binding
 * status ({@link Frame}) of the globals of each module, and of the locals of
 * the function being executed. This is stored in the {@link FixpointInfo} of
 * the analysis states, under key {@link #INFO_KEY}. <br>
 * <br>
 * Being a {@link ScopedObject}, instances of this class react to calls (see
 * {@link FixpointInfo#pushScope(ScopeToken, ProgramPoint)}): when a call is
 * made, the frame of the locals of the caller is saved, and the callee starts
 * with an empty frame; when the call returns, the frame of the caller is
 * restored. The frames of the modules are instead shared between the caller and
 * the callee, so that bindings of globals made by the callee are visible after
 * the call. The saved frames are indexed by the {@link ScopeToken} of the call:
 * if a scope is pushed with a token that is already used (e.g. a recursive call
 * through the same call site), the saved frames are merged, so that the number
 * of saved frames is bounded by the number of call sites. Instances of this
 * class are immutable.
 */
public class Namespaces implements BaseLattice<Namespaces>, ScopedObject<Namespaces> {

	/**
	 * The key of the {@link FixpointInfo} where instances of this class are
	 * stored.
	 */
	public static final String INFO_KEY = "py-namespaces";

	/**
	 * The namespaces where no name is bound.
	 */
	public static final Namespaces EMPTY = new Namespaces(Collections.emptyMap(), Frame.EMPTY,
			Collections.emptyMap());

	/**
	 * The top namespaces, where each name might be bound or not.
	 */
	public static final Namespaces TOP = new Namespaces(Collections.emptyMap(), Frame.TOP, Collections.emptyMap());

	/**
	 * The bottom namespaces, used for unreachable code.
	 */
	public static final Namespaces BOTTOM = new Namespaces(Collections.emptyMap(), Frame.BOTTOM,
			Collections.emptyMap());

	// the frames of the globals of the modules (a missing module is
	// Frame.EMPTY)
	private final Map<String, Frame> modules;

	// the frame of the locals of the function being executed
	private final Frame locals;

	// the frames of the locals of the callers, by call
	private final Map<ScopeToken, Frame> saved;

	private Namespaces(
			Map<String, Frame> modules,
			Frame locals,
			Map<ScopeToken, Frame> saved) {
		this.modules = modules;
		this.locals = locals;
		this.saved = saved;
	}

	private static Namespaces mk(
			Map<String, Frame> modules,
			Frame locals,
			Map<ScopeToken, Frame> saved) {
		// frames of modules that are empty are not stored
		Map<String, Frame> mods = new HashMap<>();
		for (Entry<String, Frame> e : modules.entrySet())
			if (e.getValue().isBottom())
				return BOTTOM;
			else if (!e.getValue().equals(Frame.EMPTY))
				mods.put(e.getKey(), e.getValue());
		if (locals.isBottom())
			return BOTTOM;
		for (Frame f : saved.values())
			if (f.isBottom())
				return BOTTOM;
		return new Namespaces(Collections.unmodifiableMap(mods), locals,
				Collections.unmodifiableMap(new HashMap<>(saved)));
	}

	/**
	 * Yields the namespaces stored in the {@link FixpointInfo} of the normal
	 * execution of the given state.
	 *
	 * @param state the state
	 *
	 * @return the namespaces, or {@code null} if the state does not contain
	 *             them
	 */
	public static Namespaces of(
			AnalysisState<?> state) {
		return state.getExecutionInfo(INFO_KEY, Namespaces.class);
	}

	/**
	 * Yields a copy of the given state where the given namespaces are stored in
	 * the {@link FixpointInfo} of the normal execution, replacing the existing
	 * ones.
	 *
	 * @param <A>        the kind of {@link AbstractLattice} of the state
	 * @param state      the state
	 * @param namespaces the namespaces
	 *
	 * @return the updated state
	 */
	public static <A extends AbstractLattice<A>> AnalysisState<A> store(
			AnalysisState<A> state,
			Namespaces namespaces) {
		return state.storeExecutionInfo(INFO_KEY, namespaces);
	}

	/**
	 * Yields the frame of the globals of the given module.
	 *
	 * @param module the name of the module
	 *
	 * @return the frame of the module
	 */
	public Frame getModule(
			String module) {
		if (isBottom())
			return Frame.BOTTOM;
		if (isTop())
			return Frame.TOP;
		return modules.getOrDefault(module, Frame.EMPTY);
	}

	/**
	 * Yields the frame of the locals of the function being executed.
	 *
	 * @return the frame of the locals
	 */
	public Frame getLocals() {
		return locals;
	}

	/**
	 * Yields the binding status of a name.
	 *
	 * @param module the name of the module the name belongs to
	 * @param name   the name
	 * @param global whether the name is a global of the module, or a local of
	 *                   the function being executed
	 *
	 * @return the binding status of the name
	 */
	public Binding status(
			String module,
			String name,
			boolean global) {
		return (global ? getModule(module) : locals).status(name);
	}

	/**
	 * Yields a copy of these namespaces where the given name is bound.
	 *
	 * @param module the name of the module the name belongs to
	 * @param name   the name
	 * @param global whether the name is a global of the module, or a local of
	 *                   the function being executed
	 *
	 * @return the updated namespaces
	 */
	public Namespaces bind(
			String module,
			String name,
			boolean global) {
		if (global)
			return withModule(module, getModule(module).bind(name));
		return withLocals(locals.bind(name));
	}

	/**
	 * Yields a copy of these namespaces where the given name is not bound.
	 *
	 * @param module the name of the module the name belongs to
	 * @param name   the name
	 * @param global whether the name is a global of the module, or a local of
	 *                   the function being executed
	 *
	 * @return the updated namespaces
	 */
	public Namespaces unbind(
			String module,
			String name,
			boolean global) {
		if (global)
			return withModule(module, getModule(module).unbind(name));
		return withLocals(locals.unbind(name));
	}

	/**
	 * Yields a copy of these namespaces where any global of the given module
	 * might be bound (see {@link Frame#havoc()}).
	 *
	 * @param module the name of the module
	 *
	 * @return the updated namespaces
	 */
	public Namespaces havocModule(
			String module) {
		return withModule(module, getModule(module).havoc());
	}

	/**
	 * Yields a copy of these namespaces where the frame of the given module is
	 * replaced with the given one.
	 *
	 * @param module the name of the module
	 * @param frame  the frame of the module
	 *
	 * @return the updated namespaces
	 */
	public Namespaces withModule(
			String module,
			Frame frame) {
		if (isBottom() || isTop())
			return this;
		Map<String, Frame> mods = new HashMap<>(modules);
		mods.put(module, frame);
		return mk(mods, locals, saved);
	}

	/**
	 * Yields a copy of these namespaces where the frame of the locals is
	 * replaced with the given one.
	 *
	 * @param frame the frame of the locals
	 *
	 * @return the updated namespaces
	 */
	public Namespaces withLocals(
			Frame frame) {
		if (isBottom() || isTop())
			return this;
		return mk(modules, frame, saved);
	}

	@Override
	public Namespaces pushScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		if (isBottom() || isTop())
			return this;
		Map<ScopeToken, Frame> s = new HashMap<>(saved);
		Frame previous = s.get(token);
		s.put(token, previous == null ? locals : previous.lub(locals));
		return mk(modules, Frame.EMPTY, s);
	}

	@Override
	public Namespaces popScope(
			ScopeToken token,
			ProgramPoint pp)
			throws SemanticException {
		if (isBottom() || isTop())
			return this;
		Map<ScopeToken, Frame> s = new HashMap<>(saved);
		Frame restored = s.remove(token);
		// a missing frame should not happen: we soundly assume that any name
		// might be bound
		return mk(modules, restored == null ? Frame.TOP : restored, s);
	}

	@Override
	public Namespaces lubAux(
			Namespaces other)
			throws SemanticException {
		Map<String, Frame> mods = new HashMap<>();
		for (String module : moduleNames(other))
			mods.put(module, getModule(module).lub(other.getModule(module)));
		Map<ScopeToken, Frame> s = new HashMap<>(saved);
		for (Entry<ScopeToken, Frame> e : other.saved.entrySet())
			s.merge(e.getKey(), e.getValue(), (
					f1,
					f2) -> lub(f1, f2));
		return mk(mods, locals.lub(other.locals), s);
	}

	@Override
	public Namespaces wideningAux(
			Namespaces other)
			throws SemanticException {
		// frames have finite height, as the names of a program are finite
		return lubAux(other);
	}

	@Override
	public Namespaces glbAux(
			Namespaces other)
			throws SemanticException {
		Map<String, Frame> mods = new HashMap<>();
		for (String module : moduleNames(other))
			mods.put(module, getModule(module).glb(other.getModule(module)));
		Map<ScopeToken, Frame> s = new HashMap<>();
		for (Entry<ScopeToken, Frame> e : saved.entrySet())
			if (other.saved.containsKey(e.getKey()))
				s.put(e.getKey(), e.getValue().glb(other.saved.get(e.getKey())));
		return mk(mods, locals.glb(other.locals), s);
	}

	@Override
	public boolean lessOrEqualAux(
			Namespaces other)
			throws SemanticException {
		for (String module : moduleNames(other))
			if (!getModule(module).lessOrEqual(other.getModule(module)))
				return false;
		if (!locals.lessOrEqual(other.locals))
			return false;
		for (Entry<ScopeToken, Frame> e : saved.entrySet()) {
			Frame o = other.saved.get(e.getKey());
			if (o == null || !e.getValue().lessOrEqual(o))
				return false;
		}
		return true;
	}

	private Set<String> moduleNames(
			Namespaces other) {
		Set<String> names = new HashSet<>(modules.keySet());
		names.addAll(other.modules.keySet());
		return names;
	}

	// lub of frames, wrapping the exception (frames never raise it)
	private static Frame lub(
			Frame f1,
			Frame f2) {
		try {
			return f1.lub(f2);
		} catch (SemanticException e) {
			throw new IllegalStateException("Unexpected exception during the lub of frames", e);
		}
	}

	@Override
	public Namespaces top() {
		return TOP;
	}

	@Override
	public Namespaces bottom() {
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
		return Objects.hash(modules, locals, saved, isTop(), isBottom());
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null || getClass() != obj.getClass())
			return false;
		Namespaces other = (Namespaces) obj;
		// top and bottom are singletons whose content is not meaningful
		return isTop() == other.isTop() && isBottom() == other.isBottom() && modules.equals(other.modules)
				&& locals.equals(other.locals) && saved.equals(other.saved);
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
		Map<StructuredRepresentation, StructuredRepresentation> mods = new TreeMap<>();
		for (Entry<String, Frame> e : modules.entrySet())
			mods.put(new StringRepresentation(e.getKey()), e.getValue().representation());
		fields.put("modules", new MapRepresentation(mods));
		fields.put("locals", locals.representation());
		if (!saved.isEmpty()) {
			Set<StructuredRepresentation> calls = new TreeSet<>();
			for (ScopeToken token : saved.keySet())
				calls.add(new StringRepresentation(token.toString()));
			fields.put("callers", new SetRepresentation(calls));
		}
		return new ObjectRepresentation(fields);
	}
}
