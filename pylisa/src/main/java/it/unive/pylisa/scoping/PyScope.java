package it.unive.pylisa.scoping;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import org.antlr.v4.runtime.ParserRuleContext;

/**
 * A scope of a Python program, that is, a block of code with its own
 * namespace: a module, a function (including lambdas), a class body, or a
 * comprehension. Scopes are built by {@link SymtableBuilder}, that collects
 * the names bound, read, and declared in each scope, and then resolves the
 * {@link NameKind} of each of them following Python's scoping rules.
 */
public class PyScope {

	/**
	 * The kinds of scopes.
	 */
	public enum ScopeKind {

		/**
		 * The scope of a module.
		 */
		MODULE,

		/**
		 * The scope of a function or of a lambda.
		 */
		FUNCTION,

		/**
		 * The scope of a class body.
		 */
		CLASS,

		/**
		 * The scope of a comprehension or of a generator expression.
		 */
		COMPREHENSION;
	}

	private final ScopeKind kind;

	private final String name;

	private final String module;

	private final PyScope parent;

	private final ParserRuleContext node;

	private final List<PyScope> children = new ArrayList<>();

	// the names collected while visiting the scope, in order of appearance

	private final Set<String> parameters = new LinkedHashSet<>();

	private final Set<String> bound = new LinkedHashSet<>();

	private final Set<String> read = new LinkedHashSet<>();

	private final Set<String> globals = new LinkedHashSet<>();

	private final Set<String> nonlocals = new LinkedHashSet<>();

	private boolean starImport = false;

	// the kinds of the names, available after the resolution
	private final Map<String, NameKind> symbols = new HashMap<>();

	/**
	 * Builds a scope.
	 *
	 * @param kind   the kind of the scope
	 * @param name   the name of the scope (e.g. the name of the function)
	 * @param module the name of the module the scope belongs to
	 * @param parent the enclosing scope, or {@code null} for the module
	 * @param node   the node of the parse tree defining the scope
	 */
	PyScope(
			ScopeKind kind,
			String name,
			String module,
			PyScope parent,
			ParserRuleContext node) {
		this.kind = kind;
		this.name = name;
		this.module = module;
		this.parent = parent;
		this.node = node;
		if (parent != null)
			parent.children.add(this);
	}

	/**
	 * Yields the kind of this scope.
	 *
	 * @return the kind
	 */
	public ScopeKind getKind() {
		return kind;
	}

	/**
	 * Yields the name of this scope (e.g. the name of the function, or of the
	 * module).
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * Yields the name of the module this scope belongs to.
	 *
	 * @return the name of the module
	 */
	public String getModule() {
		return module;
	}

	/**
	 * Yields the scope enclosing this one.
	 *
	 * @return the enclosing scope, or {@code null} if this is the scope of the
	 *             module
	 */
	public PyScope getParent() {
		return parent;
	}

	/**
	 * Yields the node of the parse tree defining this scope (e.g. the
	 * definition of the function).
	 *
	 * @return the node
	 */
	public ParserRuleContext getNode() {
		return node;
	}

	/**
	 * Yields the scopes directly nested in this one, in order of appearance.
	 *
	 * @return the nested scopes
	 */
	public List<PyScope> getChildren() {
		return Collections.unmodifiableList(children);
	}

	/**
	 * Yields whether this is the scope of a function, of a lambda, or of a
	 * comprehension, whose names are local to each of its executions.
	 *
	 * @return whether this is a function-like scope
	 */
	public boolean isFunctionLike() {
		return kind == ScopeKind.FUNCTION || kind == ScopeKind.COMPREHENSION;
	}

	/**
	 * Yields the names of the parameters of this scope, in order of
	 * declaration.
	 *
	 * @return the names of the parameters
	 */
	public Set<String> getParameters() {
		return Collections.unmodifiableSet(parameters);
	}

	/**
	 * Yields whether this module scope contains a {@code from m import *}
	 * statement, that might bind names that are not known statically.
	 *
	 * @return whether there is a star import
	 */
	public boolean hasStarImport() {
		return starImport;
	}

	/**
	 * Yields the kinds of all the names of this scope.
	 *
	 * @return the mapping from names to their kinds
	 */
	public Map<String, NameKind> getSymbols() {
		return Collections.unmodifiableMap(symbols);
	}

	/**
	 * Yields the kind of the given name inside this scope. Names that do not
	 * appear in this scope are considered {@link NameKind#GLOBAL_IMPLICIT}.
	 *
	 * @param name the name
	 *
	 * @return the kind of the name
	 */
	public NameKind kindOf(
			String name) {
		return symbols.getOrDefault(name, NameKind.GLOBAL_IMPLICIT);
	}

	/**
	 * Yields whether the given name is bound by this scope, that is, it is a
	 * parameter of the scope or it is bound by some statement or expression of
	 * the scope. This holds regardless of the {@link NameKind} of the name
	 * (e.g. a name declared {@code global} and assigned in a function is bound
	 * by the function).
	 *
	 * @param name the name
	 *
	 * @return whether the name is bound by this scope
	 */
	public boolean binds(
			String name) {
		return parameters.contains(name) || bound.contains(name);
	}

	/**
	 * Yields the names of this scope that are captured by nested scopes, in
	 * alphabetical order.
	 *
	 * @return the names of kind {@link NameKind#CELL}
	 */
	public SortedSet<String> getCellVariables() {
		return namesOfKind(NameKind.CELL);
	}

	/**
	 * Yields the names of this scope that refer to cells of enclosing
	 * functions, in alphabetical order.
	 *
	 * @return the names of kind {@link NameKind#FREE}
	 */
	public SortedSet<String> getFreeVariables() {
		return namesOfKind(NameKind.FREE);
	}

	private SortedSet<String> namesOfKind(
			NameKind kind) {
		SortedSet<String> result = new TreeSet<>();
		for (Map.Entry<String, NameKind> e : symbols.entrySet())
			if (e.getValue() == kind)
				result.add(e.getKey());
		return result;
	}

	@Override
	public String toString() {
		return kind.name().toLowerCase() + " " + name + " " + symbols;
	}

	// methods used by SymtableBuilder to populate and resolve the scope

	void addParameter(
			String name) {
		parameters.add(name);
	}

	void addBound(
			String name) {
		bound.add(name);
	}

	void addRead(
			String name) {
		read.add(name);
	}

	void addGlobal(
			String name) {
		globals.add(name);
	}

	void addNonlocal(
			String name) {
		nonlocals.add(name);
	}

	void setStarImport() {
		starImport = true;
	}

	boolean isParameter(
			String name) {
		return parameters.contains(name);
	}

	boolean isBound(
			String name) {
		return bound.contains(name);
	}

	boolean isRead(
			String name) {
		return read.contains(name);
	}

	boolean isGlobal(
			String name) {
		return globals.contains(name);
	}

	boolean isNonlocal(
			String name) {
		return nonlocals.contains(name);
	}

	Set<String> allNames() {
		Set<String> names = new LinkedHashSet<>();
		names.addAll(parameters);
		names.addAll(bound);
		names.addAll(read);
		names.addAll(globals);
		names.addAll(nonlocals);
		return names;
	}

	void setKind(
			String name,
			NameKind kind) {
		symbols.put(name, kind);
	}

	NameKind getResolvedKind(
			String name) {
		return symbols.get(name);
	}
}
