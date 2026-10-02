package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.events.EventQueue;
import it.unive.lisa.interprocedural.callgraph.CallGraphConstructionException;
import it.unive.lisa.interprocedural.callgraph.CallGraphEdge;
import it.unive.lisa.interprocedural.callgraph.CallGraphNode;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.lisa.program.Application;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.statement.CallTargets;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.statement.PyInstantiation;
import it.unive.pylisa.cfg.statement.PyResolvedCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The call graph of a Python program. A {@link PyCall} is resolved with
 * Python's dispatch, from the runtime types of its operands alone (see
 * {@link PyResolvedCall}); every other call is resolved as LiSA does.
 * <p>
 * Each resolution is computed once per call and array of types, and records
 * what LiSA's call graphs record: the call as a call site of each code member
 * it may reach (Python functions and library models alike), an edge from the
 * caller to each of them, and a {@link CallResolved} event, with a second one
 * for an {@link OpenCall} when a part of the call cannot be dispatched. The
 * calls that apply a resolution's targets are internal to it: they are never
 * call sites, so a call is listed once whatever the number of contexts and
 * fixpoint iterations it is analysed in.
 * </p>
 * <p>
 * Resolutions are shared by equal calls. Equality ignores whether a call has
 * a receiver or applies a decorator, though the ordering of calls does not:
 * the frontend never builds two equal calls that differ in either.
 * </p>
 */
public class PyCallGraph extends RTACallGraph {

	private Application app;

	private EventQueue events;

	private final Map<PyCall, Map<List<Set<Type>>, PyResolvedCall>> resolutions = new HashMap<>();

	private final Map<CodeMember, Set<Call>> sites = new HashMap<>();

	/**
	 * The calls that apply the targets of the resolutions: LiSA registers the
	 * calls it analyses as call sites, but these belong to the Python call
	 * that is resolved, which is the call site.
	 */
	private final Set<Call> applications = Collections.newSetFromMap(new IdentityHashMap<>());

	/**
	 * The classes, and the targets of {@code __new__}, of the instantiations
	 * made at each location. The calls an instantiation makes of
	 * {@code __new__} and {@code __init__} are equal to those of another
	 * instantiation at the same location whenever their owners are the same,
	 * so their operands share one stored state.
	 */
	private final Map<CodeLocation, Creators> creators = new HashMap<>();

	private record Creators(Set<Type> classes, Set<CallTargets.Target> newTargets) {
	}

	@Override
	public void init(
			Application app,
			EventQueue events)
			throws CallGraphConstructionException {
		super.init(app, events);
		this.app = app;
		this.events = events;
		resolutions.clear();
		sites.clear();
		applications.clear();
		creators.clear();
	}

	@Override
	public void registerCall(
			CFGCall call) {
		if (!applications.contains(call))
			super.registerCall(call);
	}

	@Override
	public Call resolve(
			UnresolvedCall call,
			Set<Type>[] types,
			SymbolAliasing aliasing)
			throws CallResolutionException {
		if (!(call instanceof PyCall site))
			return super.resolve(call, types, aliasing);
		Map<List<Set<Type>>, PyResolvedCall> byTypes = resolutions.computeIfAbsent(site, c -> new HashMap<>());
		List<Set<Type>> key = Arrays.asList(types);
		PyResolvedCall known = byTypes.get(key);
		if (known != null) {
			noteCreator(site, known);
			return known;
		}
		PyResolvedCall resolved = new PyResolvedCall(site, types);
		resolved.setSource(call);
		byTypes.put(key, resolved);
		applications.addAll(resolved.applications());
		noteCreator(site, resolved);
		CallGraphNode caller = node(call.getCFG());
		for (CodeMember target : resolved.getTargets()) {
			addEdge(new CallGraphEdge(caller, node(target)));
			sites.computeIfAbsent(target, member -> new LinkedHashSet<>()).add(call);
		}
		if (events != null) {
			events.post(new CallResolved(call, types, aliasing, resolved));
			// the parts that cannot be dispatched are reported as LiSA reports
			// the calls it cannot resolve; the analysis still continues them
			// as the resolved call does, not with LiSA's open call policy
			if (!resolved.unresolved().isEmpty())
				events.post(new CallResolved(call, types, aliasing, open(site)));
		}
		return resolved;
	}

	/**
	 * Records the class of the instantiation a call of {@code __new__} or
	 * {@code __init__} belongs to, and the targets of {@code __new__}.
	 */
	private void noteCreator(
			PyCall call,
			PyResolvedCall resolved) {
		if (!(call.getParentStatement() instanceof PyInstantiation instantiation))
			return;
		Creators at = creators.computeIfAbsent(call.getLocation(),
				location -> new Creators(new HashSet<>(), new HashSet<>()));
		at.classes().add(instantiation.getClassType());
		// an instantiation passes the object it creates to __init__ as its
		// receiver, and calls __new__ without one; every way __new__ may
		// create the object counts, a class it instantiates included
		if (!call.hasReceiver())
			at.newTargets().addAll(resolved.targets());
	}

	private static OpenCall open(
			PyCall call) {
		OpenCall open = new OpenCall(call);
		open.setSource(call);
		open.setParentStatement(call);
		return open;
	}

	private CallGraphNode node(
			CodeMember member) {
		CallGraphNode node = new CallGraphNode(this, member);
		if (!containsNode(node))
			addNode(node, app.getEntryPoints().contains(member));
		return node;
	}

	/**
	 * Yields the operand of a Python call that is bound to the receiver of one of its targets, for
	 * readers of the object a method is called on. A Python call passes the receiver it is written
	 * with, {@code receiver.attribute(arguments)}, unless the receiver is a module (see
	 * {@link CallTargets#receiverPassed}); a method reached through its class, as in
	 * {@code Cls.method(obj)}, is passed the class, which is not the object it is called on. The
	 * operand is the receiver only if, in every resolution of the call that reaches the target, every
	 * value of it may be an object: no module, no class, and no value of unknown type, which may be
	 * either. A reader applies the answer to every context of the call, so one resolution where the
	 * operand is not the receiver is enough to answer that no operand is.
	 *
	 * @param call   a call site of the target
	 * @param target the target
	 *
	 * @return the operand, or {@code null} if it is not certainly the object the target is called on
	 *             in every resolution, if {@code call} is the {@code __init__} of an instantiation
	 *             whose location creates objects through more than one class or more than one
	 *             target of {@code __new__}, or if {@code call} is not a Python call
	 */
	public Expression receiverOf(
			Call call,
			CodeMember target) {
		if (!(call instanceof PyCall site) || !site.hasReceiver() || site.getSubExpressions().length < 2
				|| !resolutions.containsKey(site))
			return null;
		// the object an instantiation creates is read in the one state its
		// receiver operand stores, which the last class or the last target
		// of __new__ to run leaves: with several, it is not the object of
		// every path
		if (site.getParentStatement() instanceof PyInstantiation) {
			Creators at = creators.get(site.getLocation());
			if (at == null || at.classes().size() > 1 || at.newTargets().size() > 1)
				return null;
		}
		boolean reached = false;
		for (Map.Entry<List<Set<Type>>, PyResolvedCall> resolution : resolutions.get(site).entrySet()) {
			if (!resolution.getValue().getTargets().contains(target))
				continue;
			reached = true;
			Set<Type> receiver = resolution.getKey().get(1);
			if (receiver.isEmpty() || !receiver.stream().allMatch(PyCallGraph::object))
				return null;
		}
		return reached ? site.getSubExpressions()[1] : null;
	}

	/**
	 * Whether a value of the type may be an object a method is called on: not a module, not a class,
	 * and of a type the analysis knows.
	 */
	private static boolean object(
			Type type) {
		return !(type instanceof PyModuleType) && !(type instanceof PyClassType) && !NoInfoType.INSTANCE.equals(type);
	}

	@Override
	public Collection<Call> getCallSites(
			CodeMember member) {
		Set<Call> python = sites.get(member);
		if (python == null)
			return super.getCallSites(member);
		Set<Call> all = new LinkedHashSet<>(super.getCallSites(member));
		all.addAll(python);
		return all;
	}
}
