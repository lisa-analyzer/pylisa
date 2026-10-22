package it.unive.pylisa.analysis.namespaces;

import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.CallAwareObject;
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
 * of saved frames is bounded by the number of call sites.<br>
 * <br>
 * Being a {@link CallAwareObject}, the namespaces after a call are also refined
 * with the ones of the caller (see
 * {@link #onCallReturn(Namespaces, ProgramPoint)}): the locals of the caller,
 * and the globals of the modules whose functions never bind them (see
 * {@link #withStable(String)}), are restored exactly as they were before the
 * call, regardless of the calling contexts whose results are shared by the
 * callee. Instances of this class are immutable.
 */
public class Namespaces implements BaseLattice<Namespaces>, ScopedObject<Namespaces>, CallAwareObject<Namespaces> {

	/**
	 * The key of the {@link FixpointInfo} where instances of this class are
	 * stored.
	 */
	public static final String INFO_KEY = "py-namespaces";

	/**
	 * The namespaces where no name is bound.
	 */
	public static final Namespaces EMPTY = new Namespaces(Collections.emptyMap(), Frame.EMPTY,
			Collections.emptyMap(), Collections.emptySet());

	/**
	 * The top namespaces, where each name might be bound or not.
	 */
	public static final Namespaces TOP = new Namespaces(Collections.emptyMap(), Frame.TOP, Collections.emptyMap(),
			Collections.emptySet());

	/**
	 * The bottom namespaces, used for unreachable code.
	 */
	public static final Namespaces BOTTOM = new Namespaces(Collections.emptyMap(), Frame.BOTTOM,
			Collections.emptyMap(), Collections.emptySet());

	// the frames of the globals of the modules (a missing module is
	// Frame.EMPTY)
	private final Map<String, Frame> modules;

	// the frame of the locals of the function being executed
	private final Frame locals;

	// the frames of the locals of the callers, by call
	private final Map<ScopeToken, Frame> saved;

	// the modules whose globals are never bound by functions (see
	// withStable)
	private final Set<String> stable;

	private Namespaces(
			Map<String, Frame> modules,
			Frame locals,
			Map<ScopeToken, Frame> saved,
			Set<String> stable) {
		this.modules = modules;
		this.locals = locals;
		this.saved = saved;
		this.stable = stable;
	}

	private static Namespaces mk(
			Map<String, Frame> modules,
			Frame locals,
			Map<ScopeToken, Frame> saved,
			Set<String> stable) {
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
				Collections.unmodifiableMap(new HashMap<>(saved)), Collections.unmodifiableSet(new HashSet<>(stable)));
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
		return mk(mods, locals, saved, stable);
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
		return mk(modules, frame, saved, stable);
	}

	/**
	 * Yields a copy of these namespaces where the given module is
	 * <i>stable</i>, that is, its globals are never bound or deleted by the
	 * functions of the program (e.g. through {@code global} declarations, or by
	 * executing code with {@code exec}). The frame of a stable module does not
	 * change during a call, so that it is restored from the caller when the
	 * call returns (see {@link #onCallReturn(Namespaces, ProgramPoint)}).
	 *
	 * @param module the name of the module
	 *
	 * @return the updated namespaces
	 */
	public Namespaces withStable(
			String module) {
		if (isBottom() || isTop() || stable.contains(module))
			return this;
		Set<String> st = new HashSet<>(stable);
		st.add(module);
		return mk(modules, locals, saved, st);
	}

	/**
	 * Yields whether the given module is stable (see
	 * {@link #withStable(String)}).
	 *
	 * @param module the name of the module
	 *
	 * @return whether the module is stable
	 */
	public boolean isStable(
			String module) {
		return stable.contains(module);
	}

	/**
	 * Yields the namespaces after returning from a call, given that these are
	 * the namespaces at the end of the callee, and the given ones are the ones
	 * of the caller before the call. The locals (and the saved frames of the
	 * callers) are the ones of the caller, that cannot be modified by the
	 * callee, as the frames of the stable modules (see
	 * {@link #withStable(String)}). The frames of the other modules are the
	 * ones of the callee, that might have bound or deleted their globals.<br>
	 * <br>
	 * Restoring these frames from the caller avoids the loss of precision due
	 * to the callee's results being shared by different calling contexts (e.g.
	 * with a limited context sensitivity), where the same globals might be
	 * bound or not.
	 */
	@Override
	public Namespaces onCallReturn(
			Namespaces caller,
			ProgramPoint call)
			throws SemanticException {
		if (isBottom() || isTop() || caller.isBottom() || caller.isTop())
			return this;
		Map<String, Frame> mods = new HashMap<>();
		for (String module : moduleNames(caller))
			mods.put(module, (caller.stable.contains(module) ? caller : this).getModule(module));
		Set<String> st = new HashSet<>(stable);
		st.retainAll(caller.stable);
		return mk(mods, caller.locals, caller.saved, st);
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
		return mk(modules, Frame.EMPTY, s, stable);
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
		return mk(modules, restored == null ? Frame.TOP : restored, s, stable);
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
		Set<String> st = new HashSet<>(stable);
		st.retainAll(other.stable);
		return mk(mods, locals.lub(other.locals), s, st);
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
		Set<String> st = new HashSet<>(stable);
		st.addAll(other.stable);
		return mk(mods, locals.glb(other.locals), s, st);
	}

	@Override
	public boolean lessOrEqualAux(
			Namespaces other)
			throws SemanticException {
		if (!stable.containsAll(other.stable))
			return false;
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
		return Objects.hash(modules, locals, saved, stable, isTop(), isBottom());
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
				&& locals.equals(other.locals) && saved.equals(other.saved) && stable.equals(other.stable);
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
