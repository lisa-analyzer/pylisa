package it.unive.pylisa.triage;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.triage.model.HeapModel;
import it.unive.lisa.triage.model.HeapWrites;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAttributeAccess;
import it.unive.pylisa.cfg.expression.PyClosureCreation;
import it.unive.pylisa.cfg.expression.PyListComprehension;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.cfg.statement.PyCellsInit;
import it.unive.pylisa.libraries.collections.NamedTupleBuiltin;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.dynamic.Eval;
import it.unive.pylisa.libraries.dynamic.Exec;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The contents of the heap that the code of the members of a Python program
 * writes, as the verification of the warnings uses them (see
 * {@link HeapWrites}). The fields of Python objects are their attributes, and
 * the internal fields of the models of builtin objects (e.g., the length of a
 * list); the cells are the elements of lists and the entries of dictionaries.
 * The attributes of the program are the names that its code assigns as
 * attributes ({@code o.name = v}), or binds as names (that might be fields of
 * the cells of closures, also written when functions start and when closures
 * are created), except the names that string literals might give to attributes
 * written by other means (e.g., the fields of named tuples): a function or
 * module writes
 * <ul>
 * <li>the attributes of the program it assigns or binds;</li>
 * <li>all the other fields (internal ones, and attributes that the program
 * never assigns, e.g., written by the models of builtins when creating objects,
 * such as the fields of named tuples), and all the cells, as its statements and
 * the natives it calls write them.</li>
 * </ul>
 * The natives of the builtins (e.g., of lists, dictionaries and strings) write
 * no attribute of the program; the writes of the other natives (e.g., of
 * {@code exec}, or of the models of other libraries) are unknown. The calls
 * that natives perform on behalf of the program (e.g., calling the functions
 * passed to them) are part of the call graph, so the writes of their targets
 * are found through it.<br>
 * <br>
 * The elements of lists and the entries of dictionaries at unknown positions
 * ({@link ListCreation#UNKNOWN_INDEX}, {@link DictModel#UNKNOWN_KEY},
 * {@link DictModel#UNTRACKED_KEYS}) stand for several cells, and are summaries
 * (see {@link HeapWrites#summaryChildren()}). Python objects have no property
 * of arrays (an attribute named {@code length} is mutable).
 */
public class PyHeapWrites
		implements
		HeapWrites {

	/**
	 * The packages of the natives modeling the builtins, that write no
	 * attribute of the program.
	 */
	private static final Set<String> BUILTIN_PACKAGES = Set.of("it.unive.pylisa.libraries",
			"it.unive.pylisa.libraries.bytes", "it.unive.pylisa.libraries.collections",
			"it.unive.pylisa.libraries.conversions", "it.unive.pylisa.libraries.dicts",
			"it.unive.pylisa.libraries.floats", "it.unive.pylisa.libraries.functools",
			"it.unive.pylisa.libraries.ints", "it.unive.pylisa.libraries.itertools",
			"it.unive.pylisa.libraries.lists", "it.unive.pylisa.libraries.math",
			"it.unive.pylisa.libraries.random", "it.unive.pylisa.libraries.sets", "it.unive.pylisa.libraries.strings",
			"it.unive.pylisa.libraries.svverifier");

	/**
	 * The natives running unknown code.
	 */
	private static final Set<Class<?>> UNKNOWN_CODE = Set.of(Exec.class, Eval.class);

	/**
	 * The fields of the models of builtin objects whose names might be the ones
	 * of attributes: they are always written (e.g., when creating objects).
	 */
	private static final Set<String> INTERNAL = Set.of("length", "__class__", "__self__", "__func__", "mapping",
			"start", "stop", "step");

	// the unknown index of lists and the unknown key of dictionaries might
	// have the same name
	private static final Set<String> SUMMARIES = Collections.unmodifiableSet(new HashSet<>(
			List.of(ListCreation.UNKNOWN_INDEX, DictModel.UNKNOWN_KEY, DictModel.UNTRACKED_KEYS)));

	private final Map<CodeMember, Written> cache = new HashMap<>();

	/**
	 * The attributes of each program, with the contents written by any member
	 * (all the fields that are not attributes of the program, and the cells).
	 */
	private final Map<Program, Attributes> attributes = new IdentityHashMap<>();

	@Override
	public synchronized Written writtenBy(
			CodeMember member) {
		if (cache.containsKey(member))
			return cache.get(member);
		Written written = compute(member);
		cache.put(member, written);
		return written;
	}

	@Override
	public Set<String> summaryChildren() {
		return SUMMARIES;
	}

	@Override
	public Set<String> arrayProperties() {
		return Collections.emptySet();
	}

	private Written compute(
			CodeMember member) {
		Program program = member.getDescriptor().getUnit().getProgram();
		Attributes known = attributes.computeIfAbsent(program, Attributes::new);
		if (member instanceof NativeCFG) {
			Class<?> construct = ((NativeCFG) member).getConstruct();
			if (UNKNOWN_CODE.contains(construct) || construct.getPackage() == null
					|| !BUILTIN_PACKAGES.contains(construct.getPackage().getName()))
				return null;
			return known.anyMember;
		}
		if (!(member instanceof CFG))
			return null;
		Set<String> own = names((CFG) member);
		own.retainAll(known.names);
		if (own.isEmpty())
			return known.anyMember;
		return new Written(field -> own.contains(simpleName(field)), true).union(known.anyMember);
	}

	/**
	 * The names that the code of the given cfg assigns as attributes, or binds.
	 */
	private static Set<String> names(
			CFG cfg) {
		Set<String> names = new HashSet<>();
		// every statement and sub-expression of the cfg
		cfg.accept(new GraphVisitor<CFG, Statement, Edge, Void>() {

			@Override
			public boolean visit(
					Void tool,
					CFG graph,
					Statement node) {
				if (node instanceof PyAttributeAccess && ((PyAttributeAccess) node).isWriteTarget())
					names.add(((PyAttributeAccess) node).getName());
				else if (node instanceof PyNameRef && ((PyNameRef) node).isTarget())
					names.add(((PyNameRef) node).getName());
				// the cells of closures are written also when the functions
				// start, and when the closures are created
				else if (node instanceof PyCellsInit)
					names.addAll(((PyCellsInit) node).getParameters());
				else if (node instanceof PyClosureCreation)
					for (PyClosureCreation.FreeVariable free : ((PyClosureCreation) node).getFree())
						names.add(free.name());
				else if (node instanceof PyListComprehension)
					for (PyListComprehension.Clause clause : ((PyListComprehension) node).getClauses())
						names.addAll(clause.cells().keySet());
				return true;
			}
		}, null);
		return names;
	}

	/**
	 * The names that string literals of the code of the given cfg might give to
	 * attributes, that the program writes without assigning them (e.g., the
	 * fields of named tuples, written when the tuples are created: their names
	 * are listed by string literals, see {@link NamedTupleBuiltin}).
	 */
	private static Set<String> literalNames(
			CFG cfg) {
		Set<String> names = new HashSet<>();
		cfg.accept(new GraphVisitor<CFG, Statement, Edge, Void>() {

			@Override
			public boolean visit(
					Void tool,
					CFG graph,
					Statement node) {
				if (node instanceof StringLiteral)
					for (String part : ((StringLiteral) node).getValue().split("[\\s,]+"))
						if (!part.isEmpty())
							names.add(part);
				return true;
			}
		}, null);
		return names;
	}

	/**
	 * Returns the name of the field with the given name, without the class
	 * qualifying it (see {@link HeapModel#QUALIFIER}).
	 */
	private static String simpleName(
			String field) {
		int sep = field.lastIndexOf(HeapModel.QUALIFIER);
		return sep < 0 ? field : field.substring(sep + HeapModel.QUALIFIER.length());
	}

	/**
	 * The attributes of a program.
	 */
	private static final class Attributes {

		/**
		 * The names that the code of the program assigns as attributes, or
		 * binds, except the internal fields.
		 */
		private final Set<String> names = new HashSet<>();

		/**
		 * The contents written by any member: the fields that are not
		 * attributes of the program, and the cells (a single test, shared by
		 * all the members, so that the unions of their writes keep it once).
		 */
		private final Written anyMember = new Written(field -> !names.contains(simpleName(field)), true);

		private Attributes(
				Program program) {
			Set<String> literals = new HashSet<>();
			for (CFG cfg : program.getAllCFGs()) {
				names.addAll(names(cfg));
				literals.addAll(literalNames(cfg));
			}
			names.removeAll(INTERNAL);
			names.removeIf(n -> n.startsWith("$"));
			// the names that are written by other means
			names.removeAll(literals);
		}
	}
}
