package it.unive.pylisa.frontend;

import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.Subscript_Context;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestOrStarContext;
import it.unive.pylisa.antlr.Python3Parser.TestlistContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_compContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.antlr.v4.runtime.ParserRuleContext;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public abstract class PyFrontendBase extends Python3ParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	protected static final String SELF_PARAM_NAME = "$self";

	protected static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	protected static final Logger log = LogManager.getLogger(PyFrontendBase.class);

	/**
	 * Mutable parsing state shared across every visitor. Introduced in Chunk 1
	 * of the front-end refactor; replaces the protected fields that used to
	 * live here and leak across the four-level inheritance chain.
	 */
	protected final ParserContext ctx = new ParserContext();

	/**
	 * Python program file path.
	 */
	protected final String filePath;

	protected boolean currentFileIsPackage = false;
	protected CFG init;

	/**
	 * Whether or not {@link #filePath} points to a Jupyter notebook file
	 */
	protected final boolean notebook;

	/**
	 * When {@code true}, per-statement exceptions in {@code visitFile_input}
	 * and parameter parsing are caught and logged rather than propagated. Used
	 * when loading project sub-modules so a single unsupported construct does
	 * not prevent the rest of the module from being loaded.
	 */
	protected boolean continueOnUnsupportedStatement = false;

	public PyFrontendBase setContinueOnUnsupportedStatement(
			boolean value) {
		this.continueOnUnsupportedStatement = value;
		return this;
	}

	/**
	 * List of the indexes of cells of a Jupyter notebook in the order they are
	 * to be executed. Only valid if {@link #notebook} is {@code true}.
	 */
	protected final List<Integer> cellOrder;

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath file path to a Python program
	 * @param notebook whether or not {@code filePath} points to a Jupyter
	 *                     notebook file
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook) {
		this(filePath, notebook, Collections.emptyList());
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder sequence of the indexes of cells of a Jupyter notebook
	 *                      in the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		this(filePath, notebook, List.of(cellOrder));
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder list of the indexes of cells of a Jupyter notebook in
	 *                      the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this(filePath, notebook, cellOrder, null);
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath, with an explicit source root for module resolution.
	 *
	 * @param filePath   file path to a Python program
	 * @param notebook   whether or not {@code filePath} points to a Jupyter
	 *                       notebook file
	 * @param cellOrder  list of the indexes of cells of a Jupyter notebook in
	 *                       the order they are to be executed. Only valid if
	 *                       {@code notebook} is {@code true}.
	 * @param sourceRoot explicit root directory for resolving project-relative
	 *                       imports; if {@code null}, defaults to the parent of
	 *                       {@code filePath}
	 */
	public PyFrontendBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot) {
		ctx.program(new Program(new PythonFeatures(), new PythonTypeSystem()));
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
		this.currentFileIsPackage = (filePath != null && filePath.endsWith("__init__.py"));
		ctx.currentModule(new ModuleUnit(new SourceCodeLocation(filePath, 0, 0),
				ctx.program(), "__main__"));
		ctx.currentUnit(ctx.currentModule());
		makeInit(ctx.program());
		Path baseDir = (sourceRoot != null)
				? Path.of(sourceRoot)
				: (filePath != null) ? Path.of(filePath).getParent() : Path.of(".");
		ctx.importManager(new PythonModuleImportManager(ctx.program(), init, baseDir));
		ctx.program().addUnit(ctx.currentModule());
		PyModuleType.register("__main__", ctx.currentModule());
	}

	/**
	 * Returns the parsed file path.
	 *
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	protected <T> T unsupported(
			ParserRuleContext ctx,
			String description) {
		throw new UnsupportedStatementException(
				description + " at line " + getLine(ctx) + " of " + getFilePath());
	}

	protected void unsound(
			ParserRuleContext ctx,
			String description) {
		log.warn(description + " (unsound translation) at line "
				+ getLine(ctx) + " of " + getFilePath());
	}

	protected Expression makeRef(
			String name,
			CodeLocation loc) {
		if (!ctx.shouldPrependUnitAccess())
			return new VariableRef(ctx.currentCFG(), loc, name);

		// ── CASE 1: Inside a function / method body
		// ────────────────────────────
		// Python LEGB for methods:
		// L = method-local scope only (top frame). Class body scope is NOT part
		// of LEGB — a bare name in a method does NOT see class attributes.
		// E = enclosing function scopes — NOT YET IMPLEMENTED (future work).
		// G = module (global) scope — the fallback when not found locally.
		if (ctx.currentUnit() instanceof FunctionUnit) {
			if (isNameInTopLocalScope(name))
				return new VariableRef(ctx.currentCFG(), loc, name);
			// Not in local scope: fall through to module (G) with B fallback.
			if (ctx.currentModule() != null)
				return makeNameRef(name, loc);
			return new VariableRef(ctx.currentCFG(), loc, name);
		}

		// ── CASE 2: Inside a class body (not inside a method)
		// ─────────────────
		// When currentUnit is ClassUnit, parseClassBody has pushed exactly ONE
		// scope frame. isNameInVisibleLocalScope is safe here — there is only
		// the class body frame (method scope is only pushed by visitFuncdef,
		// which changes currentUnit to FunctionUnit before pushing).
		// Class-body names map to class-scoped attribute references.
		if (ctx.currentUnit() instanceof ClassUnit cu) {
			if (isNameInVisibleLocalScope(name))
				return makeScopedAttributeRef(cu, name, loc);
			if (ctx.currentModule() != null)
				return makeNameRef(name, loc);
			return new VariableRef(ctx.currentCFG(), loc, name);
		}

		// ── CASE 3: Module-level code (ModuleUnit extends CompilationUnit)
		// ─────
		if (isNameInVisibleLocalScope(name)) {
			if (ctx.currentUnit() instanceof CompilationUnit cu)
				return makeScopedAttributeRef(cu, name, loc);
			return new VariableRef(ctx.currentCFG(), loc, name);
		}

		if (ctx.currentUnit() instanceof CompilationUnit)
			return makeNameRef(name, loc);

		if (ctx.currentModule() != null)
			return makeNameRef(name, loc);

		return new VariableRef(ctx.currentCFG(), loc, name);
	}

	protected Expression makeScopedAttributeRef(
			CompilationUnit unit,
			String name,
			CodeLocation loc) {
		return new PythonScopedAttributeAccessRef(ctx.currentCFG(), loc, unit,
				new Global(loc, unit, name, false));
	}

	protected Expression makeNameRef(
			String name,
			CodeLocation loc) {
		String modName = (ctx.currentModule() != null) ? ctx.currentModule().getName() : "__main__";
		// Parse-time hint: if the current module recorded a
		// `from <X> import <name>` (or `... as name`) binding, pass the
		// qualified name (e.g. "fastapi.APIRouter") to PyNameRef for use
		// as a fallback when runtime-scope lookup fails.
		String qualified = (ctx.imports() != null) ? ctx.imports().get(name) : null;
		return new it.unive.pylisa.cfg.statement.PyNameRef(ctx.currentCFG(), loc, name, modName, qualified);
	}

	protected void enterLocalScope() {
		ctx.enterLocalScope();
	}

	protected void exitLocalScope() {
		ctx.exitLocalScope();
	}

	protected boolean isInsideLocalScope() {
		return ctx.isInsideLocalScope();
	}

	protected void declareNameInCurrentScope(
			String name) {
		ctx.declareNameInCurrentScope(name);
	}

	protected boolean isNameInVisibleLocalScope(
			String name) {
		return ctx.isNameInVisibleLocalScope(name);
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
	protected boolean isNameInTopLocalScope(
			String name) {
		return ctx.isNameInTopLocalScope(name);
	}

	@FunctionalInterface
	protected interface BinaryFactory {
		Expression make(
				PyCFG cfg,
				CodeLocation loc,
				Expression l,
				Expression r);
	}

	protected <C extends ParserRuleContext> Expression foldBinaryOp(
			List<C> operands,
			java.util.function.Function<C, Expression> sub,
			BinaryFactory factory,
			CodeLocation loc) {
		int n = operands.size();
		if (n == 1)
			return sub.apply(operands.get(0));
		Expression acc = factory.make(ctx.currentCFG(), loc,
				sub.apply(operands.get(n - 2)), sub.apply(operands.get(n - 1)));
		for (int i = n - 3; i >= 0; i--)
			acc = factory.make(ctx.currentCFG(), loc, sub.apply(operands.get(i)), acc);
		return acc;
	}

	protected static String transformToCode(
			List<String> codeList) {
		return String.join("\n", codeList) + "\n";
	}

	protected int getLine(
			ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	protected int getCol(
			ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	public SourceCodeLocation getLocation(
			ParserRuleContext ctx) {
		// Read the source file from the token's CharStream. This stays correct
		// when `loadProjectModuleFile` recursively re-enters the visitor on a
		// sub-module (config.py, logging.py, …) — those tokens come from a
		// different CharStream whose name is the sub-module path. Falling back
		// to the frontend's own filePath would stamp every sub-module node
		// with the entry file's path (the bug this guards against).
		String source = this.getFilePath();
		if (ctx != null && ctx.getStart() != null) {
			org.antlr.v4.runtime.CharStream cs = ctx.getStart().getInputStream();
			if (cs != null) {
				String csName = cs.getSourceName();
				if (csName != null && !csName.isEmpty() && !"<unknown>".equals(csName))
					source = csName;
			}
		}
		return new SourceCodeLocation(source, getLine(ctx), getCol(ctx));
	}

	protected CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, ctx.currentModule(), false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

	protected CodeMemberDescriptor buildInitModuleCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, ctx.currentModule(), false, "$init", cfgArgs);
	}

	protected CodeMemberDescriptor buildInitClassCFGDescriptor(
			CodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};
		return new CodeMemberDescriptor(loc, ctx.currentUnit(), false, "$init", cfgArgs);
	}

	protected CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl,
			Unit unit) {
		String funcName = "$call";
		PyParameter[] cfgArgs = visitParameters(funcDecl.parameters());

		return new CodeMemberDescriptor(getLocation(funcDecl), unit,
				false,
				funcName, cfgArgs);
	}

	protected void addRetNodesToCurrentCFG() {
		PyCFG currentCFG = ctx.currentCFG();
		if (currentCFG.getNodesCount() == 0) {
			// empty method, so the ret is also the entrypoint
			currentCFG.addNode(new Ret(currentCFG, currentCFG.getDescriptor().getLocation()), true);
			return;
		}

		Ret canonicalRet = null;
		Collection<Ret> extraRets = new LinkedList<>();
		for (Statement st : currentCFG.getNodes())
			if (st instanceof Ret ret)
				if (canonicalRet == null)
					canonicalRet = ret;
				else
					extraRets.add(ret);

		if (canonicalRet == null) {
			boolean hasFallthroughExit = false;
			for (Statement st : currentCFG.getNodes())
				if (!st.stopsExecution() && currentCFG.followersOf(st).isEmpty()) {
					hasFallthroughExit = true;
					break;
				}

			// If every path already ends with an explicit stopping statement
			// (eg,
			// Return), we do not force-create an extra synthetic Ret.
			if (!hasFallthroughExit)
				return;

			canonicalRet = new Ret(currentCFG, currentCFG.getDescriptor().getLocation());
			currentCFG.addNode(canonicalRet);
		}

		// Merge all return exits to a single terminal node.
		for (Ret extra : extraRets) {
			Collection<Statement> preds = new LinkedList<>(currentCFG.predecessorsOf(extra));
			for (Statement pred : preds) {
				boolean hasNonRetFollower = currentCFG.followersOf(pred).stream()
						.anyMatch(f -> !(f instanceof Ret));
				if (!hasNonRetFollower)
					currentCFG.addEdge(new SequentialEdge(pred, canonicalRet));
			}
			for (ControlFlowStructure cf : ctx.cfs())
				cf.replace(extra, canonicalRet);
			currentCFG.getNodeList().removeNode(extra);
		}

		// every non-throwing instruction that does not have a follower
		// is ending the method
		Collection<Statement> preExits = new LinkedList<>();
		for (Statement st : currentCFG.getNodes())
			if (st != canonicalRet && !st.stopsExecution() && currentCFG.followersOf(st).isEmpty())
				preExits.add(st);

		for (Statement st : preExits)
			currentCFG.addEdge(new SequentialEdge(st, canonicalRet));

		for (VariableTableEntry entry : currentCFG.getDescriptor().getVariables())
			if (preExits.contains(entry.getScopeEnd()))
				entry.setScopeEnd(canonicalRet);
	}

	private CFG makeInit(
			Program program) {
		init = new CFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, SyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}

	protected StringLiteral strip(
			CodeLocation location,
			String string) {
		PyCFG currentCFG = ctx.currentCFG();
		// ', ''', ", """
		if (string.startsWith("'''") && string.endsWith("'''"))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "'''");
		if (string.startsWith("\"\"\"") && string.endsWith("\"\"\""))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "\"\"\"");
		if (string.startsWith("'") && string.endsWith("'"))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "'");
		if (string.startsWith("\"") && string.endsWith("\""))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "\"");
		return new PyStringLiteral(currentCFG, location, string, "\"");
	}

	protected Boolean isADict(
			DictorsetmakerContext ctx) {
		return ctx == null || ctx.test().size() == 2 * ctx.COLON().size();
	}

	protected List<Pair<Expression, Expression>> extractPairsFromDict(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx.test().size() != 2 * ctx.COLON().size())
			throw new UnsupportedStatementException(
					"We support only initialization of dictonaries in the form of <key> : <value>");
		// it is a Dict
		for (int i = 0; i < ctx.COLON().size(); i++) {
			Expression left = visitTest(ctx.test(2 * i));
			Expression right = visitTest(ctx.test(2 * i + 1));
			result.add(Pair.of(left, right));
		}
		return result;
	}

	protected List<Expression> extractElementsFromSet(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < ctx.test().size(); i++) {
			Expression e = visitTest(ctx.test(i));
			result.add(e);
		}
		return result;
	}

	protected List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext ctx) {
		if (ctx.test() != null) {
			List<Expression> r = new ArrayList<>(1);
			r.add(visitTest(ctx.test()));
			return r;
		} else
			return visitTestlist(ctx.testlist());
	}

	protected List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : ctx.subscript_())
			result.add(visitSubscript_(e));
		return result;
	}

	protected List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.testOrStar() == null || ctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : ctx.testOrStar())
			result.add(visitTestOrStar(e));
		return result;
	}

	protected List<Expression> convertAssignmentsToByNameParameters(
			List<Expression> pars) {
		List<Expression> converted = new ArrayList<>(pars.size());
		for (Expression e : pars)
			if (!(e instanceof Assignment))
				converted.add(e);
			else
				converted.add(new NamedParameterExpression(e.getCFG(), e.getLocation(),
						((Assignment) e).getLeft().toString(), ((Assignment) e).getRight()));
		return converted;
	}

	// Abstract methods that subclasses must implement (needed by helpers above)
	public abstract Expression visitTest(
			TestContext ctx);

	public abstract Expression visitTestOrStar(
			TestOrStarContext ctx);

	public abstract Expression visitSubscript_(
			Subscript_Context ctx);

	public abstract List<Expression> visitTestlist(
			TestlistContext ctx);

	public abstract PyParameter[] visitParameters(
			ParametersContext ctx);
}
