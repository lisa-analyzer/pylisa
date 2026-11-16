package it.unive.pylisa.triage;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.triage.model.GlobalWrites;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.expression.PyClassMember;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.statement.PyDelName;
import it.unive.pylisa.libraries.dynamic.Eval;
import it.unive.pylisa.libraries.dynamic.Exec;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The global variables that the code of the members of a Python program
 * writes, as the verification of the warnings uses them (see
 * {@link GlobalWrites}). The global variables of a Python program are the names
 * of the namespaces of its modules ({@code <module>.<name>}, see
 * {@link PyNameRef#identifier}), and the attributes assigned through classes
 * ({@code C.name = v}, see {@link PyClassMember}). The code of a function or
 * of a module writes:
 * <ul>
 * <li>the global names that are the targets of its bindings (assignments, also
 * augmented ones, {@code for} targets, {@code def}, {@code class},
 * {@code import}, {@code except ... as}, ...), all represented by
 * {@link PyNameRef}s that are targets, and the ones it deletes
 * ({@code del});</li>
 * <li>the attributes it assigns through classes.</li>
 * </ul>
 * The global writes of code with a star import ({@code from m import *}),
 * whose bound names are not all known, are unknown. Library functions
 * (natives) write no global variable, except {@code exec} and {@code eval},
 * that might bind or delete any name of the module (see
 * {@link it.unive.pylisa.libraries.PyNative#havocGlobals}). The calls that
 * natives perform on behalf of the program (e.g., calling the functions passed
 * to them) are part of the call graph, so the global variables written by
 * their targets are found through it.
 */
public class PyGlobalWrites
		implements
		GlobalWrites {

	/**
	 * The natives that might write any global variable of their module.
	 */
	private static final Set<Class<?>> HAVOCKING = Set.of(Exec.class, Eval.class);

	private final Map<CodeMember, Set<String>> cache = new HashMap<>();

	private final Map<Program, Set<String>> globals = new IdentityHashMap<>();

	@Override
	public synchronized Set<String> writtenBy(
			CodeMember member) {
		if (cache.containsKey(member))
			return cache.get(member);
		Set<String> written = compute(member);
		cache.put(member, written);
		return written;
	}

	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * The global variables of a Python program are not declared: they are
	 * the global names (and the attributes assigned through classes) that its
	 * code reads, binds or deletes.
	 */
	@Override
	public synchronized Set<String> globalsOf(
			Program program) {
		return globals.computeIfAbsent(program, PyGlobalWrites::used);
	}

	private static Set<String> used(
			Program program) {
		Set<String> used = new LinkedHashSet<>();
		for (CFG cfg : program.getAllCFGs())
			cfg.accept(new GraphVisitor<CFG, Statement, Edge, Void>() {

				@Override
				public boolean visit(
						Void tool,
						CFG graph,
						Statement node) {
					if (node instanceof PyNameRef)
						add((PyNameRef) node, used);
					else if (node instanceof PyClassMember) {
						String variable = ((PyClassMember) node).trackedVariable();
						if (variable != null)
							used.add(variable);
					} else
						collect(node, used);
					return true;
				}
			}, null);
		return Collections.unmodifiableSet(used);
	}

	private static Set<String> compute(
			CodeMember member) {
		if (member instanceof NativeCFG)
			return HAVOCKING.contains(((NativeCFG) member).getConstruct()) ? null : Collections.emptySet();
		if (!(member instanceof CFG))
			return null;
		Set<String> written = new HashSet<>();
		boolean[] unknown = { false };
		// every statement and sub-expression of the cfg
		((CFG) member).accept(new GraphVisitor<CFG, Statement, Edge, Void>() {

			@Override
			public boolean visit(
					Void tool,
					CFG graph,
					Statement node) {
				if (!collect(node, written))
					unknown[0] = true;
				return true;
			}
		}, null);
		return unknown[0] ? null : written;
	}

	/**
	 * Adds to the given set the names of the global variables written by the
	 * given statement, yielding {@code false} if they are not known.
	 */
	private static boolean collect(
			Statement st,
			Set<String> written) {
		if (st instanceof PyNameRef && ((PyNameRef) st).isTarget())
			add((PyNameRef) st, written);
		else if (st instanceof PyDelName)
			addAll(((PyDelName) st).getNames(), written);
		else if (st instanceof Import)
			addAll(((Import) st).getBound(), written);
		else if (st instanceof FromImport) {
			if (((FromImport) st).isStar())
				return false;
			addAll(((FromImport) st).getBound(), written);
		} else if (st instanceof PyClassMember) {
			String variable = ((PyClassMember) st).writtenVariable();
			if (variable != null)
				written.add(variable);
		}
		return true;
	}

	private static void addAll(
			List<PyNameRef> names,
			Set<String> written) {
		for (PyNameRef name : names)
			add(name, written);
	}

	private static void add(
			PyNameRef name,
			Set<String> written) {
		Identifier id = name.getIdentifier();
		if (id instanceof GlobalVariable)
			written.add(id.getName());
	}
}
