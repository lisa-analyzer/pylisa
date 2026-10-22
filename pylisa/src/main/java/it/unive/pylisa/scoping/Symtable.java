package it.unive.pylisa.scoping;

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.Map;
import org.antlr.v4.runtime.ParserRuleContext;

/**
 * The symbol table of a Python module, built by {@link SymtableBuilder}. It
 * holds the tree of {@link PyScope}s of the module, and it associates each
 * occurrence of a name that is read or bound (i.e. each node of the parse tree
 * holding such a name) to the scope it belongs to, so that the {@link NameKind}
 * of the occurrence can be retrieved without tracking scopes while visiting the
 * parse tree.
 */
public class Symtable {

	private final PyScope module;

	// the scopes, indexed by the node defining them
	private final Map<ParserRuleContext, PyScope> scopes = new IdentityHashMap<>();

	// the scope of each occurrence of a name
	private final Map<ParserRuleContext, PyScope> occurrences = new IdentityHashMap<>();

	/**
	 * Builds the symbol table.
	 *
	 * @param module the scope of the module
	 */
	Symtable(
			PyScope module) {
		this.module = module;
		register(module);
	}

	/**
	 * Yields the scope of the module.
	 *
	 * @return the scope of the module
	 */
	public PyScope getModuleScope() {
		return module;
	}

	/**
	 * Yields the scope defined by the given node of the parse tree (e.g. the
	 * scope of a function, given its definition).
	 *
	 * @param node the node
	 *
	 * @return the scope defined by the node, or {@code null} if the node does
	 *             not define a scope
	 */
	public PyScope getScope(
			ParserRuleContext node) {
		return scopes.get(node);
	}

	/**
	 * Yields all the scopes defined by nodes of the parse tree, indexed by such
	 * nodes.
	 *
	 * @return the scopes
	 */
	public Map<ParserRuleContext, PyScope> getScopes() {
		return Collections.unmodifiableMap(scopes);
	}

	/**
	 * Yields the scope the given occurrence of a name belongs to.
	 *
	 * @param occurrence the node of the parse tree holding the name
	 *
	 * @return the scope of the occurrence, or {@code null} if the node is not
	 *             an occurrence of a name that is read or bound
	 */
	public PyScope getScopeOf(
			ParserRuleContext occurrence) {
		return occurrences.get(occurrence);
	}

	/**
	 * Yields the kind of the given occurrence of a name, that is, the kind of
	 * the name in the scope the occurrence belongs to.
	 *
	 * @param occurrence the node of the parse tree holding the name
	 *
	 * @return the kind of the name, or {@code null} if the node is not an
	 *             occurrence of a name that is read or bound
	 */
	public NameKind kindOf(
			ParserRuleContext occurrence) {
		PyScope scope = occurrences.get(occurrence);
		return scope == null ? null : scope.kindOf(occurrence.getText());
	}

	void register(
			PyScope scope) {
		scopes.put(scope.getNode(), scope);
	}

	void addOccurrence(
			ParserRuleContext occurrence,
			PyScope scope) {
		occurrences.put(occurrence, scope);
	}
}
