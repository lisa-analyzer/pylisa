package it.unive.pylisa.frontend;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.frontend.definition.DefinitionVisitor;
import it.unive.pylisa.frontend.expression.ExpressionVisitor;
import it.unive.pylisa.frontend.statement.StatementVisitor;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Mutable parsing state shared by the PyLiSA front-end visitors.
 * <p>
 * Introduced in Chunk 1 of the front-end refactor and extended in Chunk 2 to
 * additionally hold cross-visitor references, so the three sibling visitors
 * (expression, statement, definition) can dispatch to one another without
 * inheriting from a common base.
 */
public final class ParserContext {

	/**
	 * Canonical name of the synthesised {@code $main} function generated for
	 * notebook entry points.
	 */
	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	/**
	 * Canonical name of the implicit {@code self} parameter injected on
	 * instance methods.
	 */
	public static final String SELF_PARAM_NAME = "$self";

	/**
	 * Shared immutable sequential-edge marker reused when constructing
	 * {@link it.unive.lisa.util.datastructures.graph.code.NodeList}s.
	 */
	public static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	private static final Logger LOG = LogManager.getLogger(ParserContext.class);

	private String filePath;
	private boolean currentFileIsPackage;
	private boolean continueOnUnsupportedStatement;

	private Program program;
	private PythonModuleImportManager importManager;
	private ModuleUnit currentModule;
	private CompilationUnit objectUnit;
	private Unit currentUnit;
	private PyCFG currentCFG;
	private CFG init;
	private Collection<ControlFlowStructure> cfs;
	private Map<String, String> imports = new HashMap<>();
	private boolean shouldPrependUnitAccess = true;
	private final Deque<Set<String>> localScopes = new ArrayDeque<>();

	private ExpressionVisitor expr;
	private StatementVisitor stmt;
	private DefinitionVisitor def;

	// === lifecycle fields ===

	public String filePath() {
		return filePath;
	}

	public void filePath(
			String path) {
		this.filePath = path;
	}

	public boolean currentFileIsPackage() {
		return currentFileIsPackage;
	}

	public void currentFileIsPackage(
			boolean v) {
		this.currentFileIsPackage = v;
	}

	public boolean continueOnUnsupportedStatement() {
		return continueOnUnsupportedStatement;
	}

	public void continueOnUnsupportedStatement(
			boolean v) {
		this.continueOnUnsupportedStatement = v;
	}

	// === parser state fields ===

	public Program program() {
		return program;
	}

	public void program(
			Program p) {
		this.program = p;
	}

	public PythonModuleImportManager importManager() {
		return importManager;
	}

	public void importManager(
			PythonModuleImportManager m) {
		this.importManager = m;
	}

	public ModuleUnit currentModule() {
		return currentModule;
	}

	public void currentModule(
			ModuleUnit m) {
		this.currentModule = m;
	}

	public CompilationUnit objectUnit() {
		return objectUnit;
	}

	public void objectUnit(
			CompilationUnit u) {
		this.objectUnit = u;
	}

	public Unit currentUnit() {
		return currentUnit;
	}

	public void currentUnit(
			Unit u) {
		this.currentUnit = u;
	}

	public PyCFG currentCFG() {
		return currentCFG;
	}

	public void currentCFG(
			PyCFG c) {
		this.currentCFG = c;
	}

	public CFG init() {
		return init;
	}

	public void init(
			CFG c) {
		this.init = c;
	}

	public Collection<ControlFlowStructure> cfs() {
		return cfs;
	}

	public void cfs(
			Collection<ControlFlowStructure> c) {
		this.cfs = c;
	}

	public Map<String, String> imports() {
		return imports;
	}

	public void imports(
			Map<String, String> m) {
		this.imports = m;
	}

	public boolean shouldPrependUnitAccess() {
		return shouldPrependUnitAccess;
	}

	public void shouldPrependUnitAccess(
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

	// === cross-visitor wiring ===

	/**
	 * Registers the three sibling visitors so that each can dispatch to the
	 * others without a shared superclass. Must be called exactly once, during
	 * frontend construction.
	 */
	public void wireVisitors(
			ExpressionVisitor e,
			StatementVisitor s,
			DefinitionVisitor d) {
		this.expr = Objects.requireNonNull(e);
		this.stmt = Objects.requireNonNull(s);
		this.def = Objects.requireNonNull(d);
		LOG.debug("visitors wired");
	}

	public ExpressionVisitor expr() {
		return expr;
	}

	public StatementVisitor stmt() {
		return stmt;
	}

	public DefinitionVisitor def() {
		return def;
	}
}
