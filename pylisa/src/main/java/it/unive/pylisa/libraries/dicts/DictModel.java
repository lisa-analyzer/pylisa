package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * The model of dictionaries (see
 * {@link it.unive.pylisa.cfg.expression.DictionaryCreation}): a dictionary is
 * an object whose children are its entries. The keys of the entries created by
 * the dictionary literal are tracked when they are {@code str} or {@code int}
 * constants: the value of the key {@code k} is the child {@code k} of the
 * object. Since the entries of a dictionary change when it is modified, no
 * other child is created afterwards: the values of the other keys (added later,
 * or that are not constants) are the child {@value #UNKNOWN_KEY}, and when it
 * is set any key might be missing (as the entries might also have been removed,
 * by the operations that are not modeled).
 */
public final class DictModel {

	/**
	 * The child of a dictionary holding the values of the keys that are not
	 * tracked. It is not a valid python identifier, so that it cannot be an
	 * attribute of the dictionary.
	 */
	public static final String UNKNOWN_KEY = "$unknown";

	/**
	 * The child of a dictionary holding any of the keys that are not tracked
	 * (whose values are the child {@value #UNKNOWN_KEY}), so that iterating
	 * over the keys yields values of the right types (e.g. {@code str}s, that
	 * are hashable). As {@value #UNKNOWN_KEY}, it is not a valid python
	 * identifier.
	 */
	public static final String UNTRACKED_KEYS = "$keys";

	private DictModel() {
	}

	/**
	 * Whether the values of the given type are dictionaries.
	 *
	 * @param t the type
	 *
	 * @return whether the values are dictionaries
	 */
	public static boolean isDict(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.DICT);
	}

	/**
	 * Whether the dictionaries do not define the given method (that is, it is
	 * not modeled).
	 *
	 * @param name the name of the method
	 *
	 * @return whether the method is not modeled
	 */
	public static boolean unmodeled(
			String name) {
		PyClassType dict = PyClassType.lookup(LibrarySpecificationProvider.DICT);
		if (dict == null)
			return false;
		Set<Unit> units = new HashSet<>();
		units.add(dict.getUnit());
		units.addAll(dict.getUnit().getImmediateAncestors());
		for (Unit unit : units)
			if (!unit.getCodeMembersByName(name).isEmpty() || (unit instanceof CompilationUnit
					&& !((CompilationUnit) unit).getInstanceCodeMembersByName(name, true).isEmpty()))
				return false;
		return true;
	}

	/**
	 * The key tracked for the given key value, that is, the value of a
	 * {@code str} or {@code int} constant ({@code bool} and {@code float} keys
	 * are not tracked, as they might be equal to an {@code int} one, e.g.
	 * {@code True == 1}).
	 *
	 * @param key the key
	 *
	 * @return the tracked key, or {@code null}
	 */
	public static Object trackedKey(
			SymbolicExpression key) {
		if (!(key instanceof Constant))
			return null;
		Object value = ((Constant) key).getValue();
		return value instanceof String || value instanceof Integer ? value : null;
	}

	/**
	 * The child of a dictionary for the given tracked key.
	 *
	 * @param key the key (see {@link #trackedKey(SymbolicExpression)})
	 * @param loc the location
	 *
	 * @return the child
	 */
	public static Constant child(
			Object key,
			CodeLocation loc) {
		return key instanceof String ? new Constant(StringType.INSTANCE, key, loc)
				: new Constant(Int32Type.INSTANCE, key, loc);
	}

	// what is known about the entries of the objects a dictionary points to
	private record Entries(
			Map<AllocationSite, Set<Object>> keys,
			Set<AllocationSite> unknown,
			Set<AllocationSite> keyed,
			boolean weak) {
	}

	// the entries of the dictionary, or null if they are not known
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Entries entries(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			ProgramPoint pp)
			throws SemanticException {
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || heap.isTop() || heap.isBottom() || heap.fields.isTop() || heap.fields.function == null)
			return null;
		Set<AllocationSite> sites = new HashSet<>();
		for (SymbolicExpression rec : analysis.rewrite(state, dict, pp))
			if (rec instanceof MemoryPointer && ((MemoryPointer) rec).getReferencedLocation() instanceof AllocationSite)
				sites.add((AllocationSite) ((MemoryPointer) rec).getReferencedLocation());
			else if (rec instanceof Identifier && heap.knowsIdentifier((Identifier) rec)) {
				AllocationSites pointed = heap.getState((Identifier) rec);
				if (pointed.isTop() || pointed.isBottom())
					return null;
				sites.addAll(pointed.elements());
			} else
				return null;
		if (sites.isEmpty())
			return null;
		Map<AllocationSite, Set<Object>> keys = new HashMap<>();
		Set<AllocationSite> unknown = new HashSet<>(), keyed = new HashSet<>();
		boolean weak = false;
		for (AllocationSite site : sites) {
			ExpressionSet fields = heap.fields.getState(site);
			if (fields.isTop())
				return null;
			weak |= site.isWeak();
			Set<Object> siteKeys = new HashSet<>();
			for (SymbolicExpression field : fields)
				if (field instanceof Variable && ((Variable) field).getName().equals(UNKNOWN_KEY))
					unknown.add(site);
				else if (field instanceof Variable && ((Variable) field).getName().equals(UNTRACKED_KEYS))
					keyed.add(site);
				else if (field instanceof Constant && trackedKey(field) != null)
					siteKeys.add(trackedKey(field));
			keys.put(site, siteKeys);
		}
		return new Entries(keys, unknown, keyed, weak);
	}

	// the tracked key the key is surely equal to, if any
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Object exactKey(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression key,
			Set<Object> candidates,
			ProgramPoint pp)
			throws SemanticException {
		Object exact = trackedKey(key);
		if (exact != null)
			return exact;
		CodeLocation loc = pp.getLocation();
		for (Object candidate : candidates)
			if (analysis.satisfies(state,
					new BinaryExpression(BoolType.INSTANCE, key, child(candidate, loc), ComparisonEq.INSTANCE, loc),
					pp) == Satisfiability.SATISFIED)
				return candidate;
		return null;
	}

	// whether the values of the key might not be hashable (e.g. a list),
	// raising TypeError
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean unhashable(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression key,
			ProgramPoint pp)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, key, pp);
		return types.isEmpty() || types.stream().anyMatch(t -> ListModel.isList(t) || isDict(t));
	}

	/**
	 * Reads the value of the given key of the dictionary ({@code dict[key]}),
	 * raising {@code KeyError} if the key might be missing, and
	 * {@code TypeError} if it might not be hashable. The value is read exactly
	 * if the key is surely equal to a tracked key that the dictionary surely
	 * has.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the dictionary and the key have been
	 *                     evaluated
	 * @param dict     the dictionary
	 * @param key      the key
	 * @param thrower  the statement reading the value
	 *
	 * @return the state after the read
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> read(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression key,
			Statement thrower)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		AnalysisState<A> result = state.bottomExecution();
		if (unhashable(analysis, state, key, thrower))
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.TYPE_ERROR));

		Entries entries = entries(analysis, state, dict, thrower);
		if (entries == null)
			return result
					.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), thrower))
					.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
							LibrarySpecificationProvider.KEY_ERROR));

		Set<Object> all = new HashSet<>();
		entries.keys().values().forEach(all::addAll);
		Object exact = entries.weak() ? null : exactKey(analysis, state, key, all, thrower);
		boolean keyError = !entries.unknown().isEmpty();
		Set<SymbolicExpression> read = new HashSet<>();
		if (exact != null) {
			boolean present = false, absent = false;
			for (Set<Object> keys : entries.keys().values())
				if (keys.contains(exact))
					present = true;
				else
					absent = true;
			if (present)
				read.add(child(exact, loc));
			if (absent) {
				// the key might be one of the untracked ones
				keyError = true;
				if (!entries.unknown().isEmpty())
					read.add(new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc));
			}
		} else {
			// any of the values
			keyError = true;
			for (Object k : all)
				read.add(child(k, loc));
			if (!entries.unknown().isEmpty())
				read.add(new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc));
		}

		for (SymbolicExpression field : read)
			result = result.lub(
					analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, deref, field, loc), thrower));
		if (keyError)
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.KEY_ERROR));
		return result;
	}

	/**
	 * Writes the value of the given key of the dictionary
	 * ({@code dict[key] = value}), raising {@code TypeError} if the key might
	 * not be hashable. The value of a tracked key that the dictionary surely
	 * has is updated; any other key is not tracked (see {@link DictModel}), and
	 * a key that might be equal to a tracked one also updates (weakly) its
	 * value.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the dictionary, the key and the value
	 *                     have been evaluated
	 * @param dict     the dictionary
	 * @param key      the key
	 * @param value    the value
	 * @param thrower  the statement writing the value
	 *
	 * @return the state after the write
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> write(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression key,
			SymbolicExpression value,
			Statement thrower)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		if (unhashable(analysis, state, key, thrower))
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), thrower.getLocation(), thrower,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result.lub(store(analysis, state, dict, key, value, thrower));
	}

	// writes the value of the given key, that is hashable (see write)
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> store(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression key,
			SymbolicExpression value,
			Statement thrower)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		Entries entries = entries(analysis, state, dict, thrower);
		if (entries == null)
			// the entries of the dictionary are not known
			return state;

		Set<Object> all = new HashSet<>();
		entries.keys().values().forEach(all::addAll);
		Object exact = exactKey(analysis, state, key, all, thrower);
		AnalysisState<A> written = state;
		if (exact != null && entries.keys().values().stream().allMatch(keys -> keys.contains(exact)))
			// the value of a tracked key
			written = analysis.assign(state, new AccessChild(Untyped.INSTANCE, deref, child(exact, loc), loc),
					value, thrower);
		else {
			if (exact == null)
				// the key might be equal to any of the tracked ones
				for (Object k : all)
					written = analysis.assign(written, new AccessChild(Untyped.INSTANCE, deref, child(k, loc), loc),
							value, thrower).lub(written);
			else if (all.contains(exact))
				// tracked only for some of the dictionaries
				written = analysis.assign(written, new AccessChild(Untyped.INSTANCE, deref, child(exact, loc), loc),
						value, thrower).lub(written);
			// a key that is not tracked
			written = analysis.assign(written,
					new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc), loc),
					value, thrower).lub(written);
			written = analysis.assign(written, untrackedKeys(deref, loc), key, thrower).lub(written);
		}
		return written;
	}

	/**
	 * The length of the dictionary, that is known if the dictionary is a single
	 * object (and not the summary of many) whose keys are all tracked.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param pp       the program point using the length
	 *
	 * @return the length, or {@code null} if it is not known
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Integer length(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			ProgramPoint pp)
			throws SemanticException {
		Entries entries = entries(analysis, state, dict, pp);
		if (entries == null || entries.weak() || !entries.unknown().isEmpty())
			return null;
		Integer length = null;
		for (Set<Object> keys : entries.keys().values())
			if (length == null)
				length = keys.size();
			else if (length != keys.size())
				return null;
		return length;
	}

	/**
	 * Invalidates what is known about the entries of the dictionary, as done by
	 * the operations on dictionaries that are not modeled (e.g. {@code pop}):
	 * the values become unknown, and any key might be missing.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param pp       the program point modifying the dictionary
	 *
	 * @return the state after the invalidation
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> invalidate(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		AnalysisState<A> result = state;
		Entries entries = entries(analysis, state, dict, pp);
		if (entries != null)
			for (Set<Object> keys : entries.keys().values())
				for (Object k : keys)
					result = analysis.assign(result, new AccessChild(Untyped.INSTANCE, deref, child(k, loc), loc),
							new PushAny(Untyped.INSTANCE, loc), pp);
		result = analysis.assign(result,
				new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc), loc),
				new PushAny(Untyped.INSTANCE, loc), pp);
		// keys of any type might have been added
		return analysis.assign(result, untrackedKeys(deref, loc), new PushAny(Untyped.INSTANCE, loc), pp);
	}

	/**
	 * Adds the entries of {@code source} to the dictionary ({@code {...,
	 * **source, ...}}, or {@code dict |= source}), overriding the ones it
	 * already has. If the dictionary is being created, the keys that
	 * {@code source} surely has become tracked keys of the dictionary (see
	 * {@link DictModel}), otherwise they are written as any other key (see
	 * {@link #write}); the values of the keys {@code source} might have are
	 * weakly updated, as the ones of its untracked keys, that might be equal to
	 * any key. A {@code source} that is not a mapping raises {@code TypeError}:
	 * surely if it is a value of a builtin class that is not a mapping (e.g. an
	 * {@code int} or a {@code list}), possibly if it is an object, whose
	 * entries are then unknown.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the dictionary and the source have been
	 *                     evaluated
	 * @param dict     the dictionary
	 * @param source   the unpacked value
	 * @param thrower  the statement adding the entries
	 * @param creating whether the dictionary is being created
	 *
	 * @return the state after the entries have been added
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unpackInto(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression source,
			Statement thrower,
			boolean creating)
			throws SemanticException {
		CodeLocation loc = thrower.getLocation();
		Type dictType = PyClassType.lookup(LibrarySpecificationProvider.DICT);
		HeapDereference target = new HeapDereference(dictType, dict, loc);
		Set<Type> types = analysis.getRuntimeTypesOf(state, source, thrower);
		AnalysisState<A> result = state.bottomExecution();

		if (types.isEmpty() || types.stream().anyMatch(t -> !isDict(t))) {
			result = result.lub(PyExceptions.raise(analysis, state, thrower.getCFG(), loc, thrower,
					LibrarySpecificationProvider.TYPE_ERROR));
			boolean notMappings = !types.isEmpty() && types.stream().filter(t -> !isDict(t))
					.allMatch(t -> (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
							|| ListModel.isList(t) || t.isCodeMemberType());
			if (!notMappings)
				// e.g. an object whose class defines keys and __getitem__
				result = result.lub(anyEntry(analysis, state, dict, new PushAny(Untyped.INSTANCE, loc),
						new PushAny(Untyped.INSTANCE, loc), thrower));
		}
		if (types.stream().noneMatch(DictModel::isDict))
			return result;

		Entries entries = entries(analysis, state, source, thrower);
		if (entries == null)
			return result.lub(anyEntry(analysis, state, dict, new PushAny(Untyped.INSTANCE, loc),
					new PushAny(Untyped.INSTANCE, loc), thrower));

		HeapDereference from = new HeapDereference(dictType, source, loc);
		Set<Object> all = new HashSet<>(), sure = null;
		for (Set<Object> keys : entries.keys().values()) {
			all.addAll(keys);
			if (sure == null)
				sure = new HashSet<>(keys);
			else
				sure.retainAll(keys);
		}
		if (sure == null || entries.weak())
			// a summary of many dictionaries
			sure = Set.of();

		AnalysisState<A> current = state;
		for (Object k : all) {
			AnalysisState<A> values = analysis.smallStepSemantics(current,
					new AccessChild(Untyped.INSTANCE, from, child(k, loc), loc), thrower);
			AnalysisState<A> next = current.bottomExecution();
			for (SymbolicExpression value : values.getExecutionExpressions())
				if (sure.contains(k) && creating)
					next = next.lub(analysis.assign(values,
							new AccessChild(Untyped.INSTANCE, target, child(k, loc), loc), value, thrower));
				else if (sure.contains(k))
					next = next.lub(write(analysis, values, dict, child(k, loc), value, thrower));
				else
					// the key might be missing: the value is weakly updated
					next = next.lub(write(analysis, values, dict, child(k, loc), value, thrower).lub(current));
			current = next;
		}
		if (!entries.unknown().isEmpty()) {
			// the untracked entries of source, whose keys and values are not
			// paired
			AnalysisState<A> keys = untrackedKey(analysis, current, source, entries, thrower);
			AnalysisState<A> next = current.bottomExecution();
			for (SymbolicExpression key : keys.getExecutionExpressions()) {
				AnalysisState<A> values = analysis.smallStepSemantics(keys,
						new AccessChild(Untyped.INSTANCE, from, new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc), loc),
						thrower);
				for (SymbolicExpression value : values.getExecutionExpressions())
					next = next.lub(anyEntry(analysis, values, dict, key, value, thrower));
			}
			current = next;
		}
		return result.lub(current);
	}

	/**
	 * Adds an entry whose key is not tracked, and that might be equal to any of
	 * the keys of the dictionary: the values of all the keys are weakly
	 * updated. The key comes from a mapping, so it is hashable.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param key      the key of the entry (e.g. an unknown value)
	 * @param value    the value of the entry
	 * @param thrower  the statement adding the entry
	 *
	 * @return the state after the entry has been added
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> anyEntry(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression key,
			SymbolicExpression value,
			Statement thrower)
			throws SemanticException {
		// the key comes from a mapping: it is hashable
		return store(analysis, state, dict, key, value, thrower).lub(state);
	}

	// any of the keys that are not tracked: the ones recorded in the child
	// UNTRACKED_KEYS, if all the dictionaries with untracked entries have it
	// (and the dictionary is not a summary of many), an unknown value
	// otherwise
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> untrackedKey(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			Entries entries,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		if (entries == null || entries.weak() || entries.unknown().isEmpty()
				|| !entries.keyed().containsAll(entries.unknown()))
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		return analysis.smallStepSemantics(state, untrackedKeys(deref, loc), pp);
	}

	private static AccessChild untrackedKeys(
			HeapDereference deref,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, UNTRACKED_KEYS, loc), loc);
	}

	/**
	 * Yields any of the keys of the dictionary (e.g. the one bound to the
	 * variable of a comprehension iterating over the dictionary): the tracked
	 * keys, and an unknown value if the dictionary might have other keys.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param pp       the program point reading the keys
	 *
	 * @return the state whose computed expressions are the keys
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> keys(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Entries entries = entries(analysis, state, dict, pp);
		AnalysisState<A> result = state.bottomExecution();
		if (entries == null || entries.weak() || !entries.unknown().isEmpty())
			result = result.lub(untrackedKey(analysis, state, dict, entries, pp));
		if (entries != null)
			for (Set<Object> keys : entries.keys().values())
				for (Object k : keys)
					result = result.lub(analysis.smallStepSemantics(state, child(k, loc), pp));
		return result;
	}

	/**
	 * Yields any of the values of the dictionary (e.g. the one bound to the
	 * variable of a loop over {@code d.values()}): the values of the tracked
	 * keys, and the ones of the other keys, if the dictionary might have them.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param pp       the program point reading the values
	 *
	 * @return the state whose computed expressions are the values
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> values(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			ProgramPoint pp)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Entries entries = entries(analysis, state, dict, pp);
		if (entries == null)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp);
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		AnalysisState<A> result = state.bottomExecution();
		for (Object k : tracked(entries))
			result = result.lub(analysis.smallStepSemantics(state,
					new AccessChild(Untyped.INSTANCE, deref, child(k, loc), loc), pp));
		if (!entries.unknown().isEmpty())
			result = result.lub(analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, deref,
					new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc), loc), pp));
		return result;
	}

	/**
	 * Yields any of the items of the dictionary (e.g. the one bound to the
	 * variables of a loop over {@code d.items()}): a new tuple {@code (k, v)}
	 * for each tracked key {@code k} and its value {@code v}, and one with an
	 * unknown key and any of the values of the other keys, if the dictionary
	 * might have them. Each tuple is allocated at its own location, derived
	 * from the one of {@code pp}.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param dict     the dictionary
	 * @param pp       the statement reading the items
	 * @param metas    where the receivers of the tuples are added, as they are
	 *                     meta variables that the statement must forget (see
	 *                     {@link it.unive.pylisa.cfg.expression.TupleCreation#create})
	 *
	 * @return the state whose computed expressions are the items
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> items(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Entries entries = entries(analysis, state, dict, pp);
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.DICT), dict, loc);
		AnalysisState<A> result = state.bottomExecution();
		if (entries != null)
			for (Object k : tracked(entries)) {
				AnalysisState<A> value = analysis.smallStepSemantics(state,
						new AccessChild(Untyped.INSTANCE, deref, child(k, loc), loc), pp);
				result = result.lub(TupleCreation.create(analysis, value, pp, new DerivedLocation(loc, "item " + k),
						new ExpressionSet[] { new ExpressionSet(child(k, loc)), value.getExecutionExpressions() },
						metas));
			}
		if (entries == null || !entries.unknown().isEmpty()) {
			AnalysisState<A> value = entries == null
					? analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp)
					: analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, deref,
							new Variable(Untyped.INSTANCE, UNKNOWN_KEY, loc), loc), pp);
			AnalysisState<A> key = untrackedKey(analysis, value, dict, entries, pp);
			result = result.lub(TupleCreation.create(analysis, key, pp, new DerivedLocation(loc, "item"),
					new ExpressionSet[] { key.getExecutionExpressions(), value.getExecutionExpressions() },
					metas));
		}
		return result;
	}

	// the tracked keys of the dictionary
	private static Set<Object> tracked(
			Entries entries) {
		Set<Object> all = new HashSet<>();
		entries.keys().values().forEach(all::addAll);
		return all;
	}
}
