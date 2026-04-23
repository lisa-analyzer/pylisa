package it.unive.pylisa.frontend;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Mutable parsing state shared by the PyLiSA front-end visitors.
 * <p>
 * Introduced in Chunk 1 of the front-end refactor. Takes the place of the
 * {@code protected} fields that used to live on {@link PyFrontendBase} and leak
 * across the four-level visitor inheritance chain. All access now goes through
 * this object, so a future chunk can flatten the inheritance into sibling
 * visitors without chasing state across class boundaries.
 * </p>
 */
public final class ParserContext {

	private static final Logger LOG = LogManager.getLogger(ParserContext.class);

	private Program program;
	private PythonModuleImportManager importManager;
	private ModuleUnit currentModule;
	private CompilationUnit objectUnit;
	private Unit currentUnit;
	private PyCFG currentCFG;
	private Collection<ControlFlowStructure> cfs;
	private Map<String, String> imports = new HashMap<>();
	private boolean shouldPrependUnitAccess = true;
	private final Deque<Set<String>> localScopes = new ArrayDeque<>();

	// === field accessors (package-private — every consumer lives in
	// it.unive.pylisa.frontend) ===

	Program program() {
		return program;
	}

	void program(
			Program p) {
		this.program = p;
	}

	PythonModuleImportManager importManager() {
		return importManager;
	}

	void importManager(
			PythonModuleImportManager m) {
		this.importManager = m;
	}

	ModuleUnit currentModule() {
		return currentModule;
	}

	void currentModule(
			ModuleUnit m) {
		this.currentModule = m;
	}

	CompilationUnit objectUnit() {
		return objectUnit;
	}

	void objectUnit(
			CompilationUnit u) {
		this.objectUnit = u;
	}

	Unit currentUnit() {
		return currentUnit;
	}

	void currentUnit(
			Unit u) {
		this.currentUnit = u;
	}

	PyCFG currentCFG() {
		return currentCFG;
	}

	void currentCFG(
			PyCFG c) {
		this.currentCFG = c;
	}

	Collection<ControlFlowStructure> cfs() {
		return cfs;
	}

	void cfs(
			Collection<ControlFlowStructure> c) {
		this.cfs = c;
	}

	Map<String, String> imports() {
		return imports;
	}

	void imports(
			Map<String, String> m) {
		this.imports = m;
	}

	boolean shouldPrependUnitAccess() {
		return shouldPrependUnitAccess;
	}

	void shouldPrependUnitAccess(
			boolean v) {
		this.shouldPrependUnitAccess = v;
	}

	// === scope-stack operations (public API for visitors) ===

	public void enterLocalScope() {
		localScopes.push(new HashSet<>());
		LOG.trace("enterLocalScope depth={}", localScopes.size());
	}

	public void exitLocalScope() {
		if (!localScopes.isEmpty()) {
			localScopes.pop();
			LOG.trace("exitLocalScope depth={}", localScopes.size());
		}
	}

	public boolean isInsideLocalScope() {
		return !localScopes.isEmpty();
	}

	public void declareNameInCurrentScope(
			String name) {
		if (!localScopes.isEmpty() && name != null && !name.isEmpty())
			localScopes.peek().add(name);
	}

	public boolean isNameInVisibleLocalScope(
			String name) {
		if (name == null || name.isEmpty())
			return false;
		for (Set<String> scope : localScopes)
			if (scope.contains(name))
				return true;
		return false;
	}

	/**
	 * Returns true if {@code name} is declared in the topmost (innermost) scope
	 * frame only. Used inside function/method bodies to implement the L step of
	 * Python LEGB resolution: only the method's own local scope is considered;
	 * enclosing class-body scope frames are intentionally excluded.
	 *
	 * @param name the name to look up
	 *
	 * @return {@code true} if {@code name} is in the top scope frame
	 */
	public boolean isNameInTopLocalScope(
			String name) {
		if (name == null || name.isEmpty())
			return false;
		Set<String> top = localScopes.peek();
		return top != null && top.contains(name);
	}
}
