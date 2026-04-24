package it.unive.pylisa.frontend;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.SourceCodeLocation;
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
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import org.antlr.v4.runtime.ParserRuleContext;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Stateless helper functions shared by every category visitor (expression,
 * statement, definition). Methods may consult {@link ParserContext} but must
 * not own mutable state.
 * <p>
 * Home for the helpers that used to live on the deleted {@code PyFrontendBase}
 * class. Helpers that require cross-visitor dispatch stay on the visitor that
 * owns the called method.
 */
public final class ParserSupport {

	private static final Logger LOG = LogManager.getLogger(ParserSupport.class);

	private final ParserContext ctx;

	public ParserSupport(
			ParserContext ctx) {
		this.ctx = Objects.requireNonNull(ctx);
	}

	// === location helpers ===

	public int getLine(
			ParserRuleContext pctx) {
		return pctx.getStart().getLine();
	}

	public int getCol(
			ParserRuleContext pctx) {
		return pctx.getStop().getCharPositionInLine();
	}

	public SourceCodeLocation getLocation(
			ParserRuleContext pctx) {
		// Read the source file from the token's CharStream. This stays correct
		// when `loadProjectModuleFile` recursively re-enters the visitor on a
		// sub-module (config.py, logging.py, …) — those tokens come from a
		// different CharStream whose name is the sub-module path. Falling back
		// to the frontend's own filePath would stamp every sub-module node
		// with the entry file's path (the bug this guards against).
		String source = ctx.filePath();
		if (pctx != null && pctx.getStart() != null) {
			org.antlr.v4.runtime.CharStream cs = pctx.getStart().getInputStream();
			if (cs != null) {
				String csName = cs.getSourceName();
				if (csName != null && !csName.isEmpty() && !"<unknown>".equals(csName))
					source = csName;
			}
		}
		return new SourceCodeLocation(source, getLine(pctx), getCol(pctx));
	}

	// === diagnostics ===

	public <T> T unsupported(
			ParserRuleContext pctx,
			String description) {
		SourceCodeLocation loc = getLocation(pctx);
		ctx.reporter().report(
				DiagnosticReporter.Severity.UNSUPPORTED,
				loc,
				featureLabel(description, pctx),
				description);
		throw new UnsupportedStatementException(
				description + " at line " + getLine(pctx) + " of " + ctx.filePath());
	}

	public void unsound(
			ParserRuleContext pctx,
			String description) {
		SourceCodeLocation loc = getLocation(pctx);
		ctx.reporter().report(
				DiagnosticReporter.Severity.UNSOUND,
				loc,
				featureLabel(description, pctx),
				description + " (unsound translation) at line "
						+ getLine(pctx) + " of " + ctx.filePath());
	}

	/**
	 * Extracts a short, lower-cased feature label from a diagnostic
	 * description. Used so callers can filter events by category (e.g. "async",
	 * "return") without coupling to full message wording. Falls back to the
	 * ANTLR rule-context class name when the description is empty.
	 */
	private static String featureLabel(
			String description,
			ParserRuleContext pctx) {
		if (description != null && !description.isBlank()) {
			String first = description.trim().split("\\s+", 2)[0];
			if (!first.isEmpty())
				return first.toLowerCase();
		}
		return pctx != null ? pctx.getClass().getSimpleName() : "unknown";
	}

	// === name resolution ===

	public Expression makeRef(
			String name,
			CodeLocation loc) {
		if (!ctx.shouldPrependUnitAccess())
			return new VariableRef(ctx.currentCFG(), loc, name);

		// ── CASE 1: Inside a function / method body
		// Python LEGB for methods:
		// L = method-local scope only (top frame). Class body scope is NOT
		// part of LEGB — a bare name in a method does NOT see class
		// attributes.
		// E = enclosing function scopes — NOT YET IMPLEMENTED (future work).
		// G = module (global) scope — the fallback when not found locally.
		if (ctx.currentUnit() instanceof FunctionUnit) {
			if (ctx.isNameInTopLocalScope(name))
				return new VariableRef(ctx.currentCFG(), loc, name);
			if (ctx.currentModule() != null)
				return makeNameRef(name, loc);
			return new VariableRef(ctx.currentCFG(), loc, name);
		}

		// ── CASE 2: Inside a class body (not inside a method)
		// When currentUnit is ClassUnit, parseClassBody has pushed exactly ONE
		// scope frame. isNameInVisibleLocalScope is safe here — there is only
		// the class body frame (method scope is only pushed by visitFuncdef,
		// which changes currentUnit to FunctionUnit before pushing).
		if (ctx.currentUnit() instanceof ClassUnit cu) {
			if (ctx.isNameInVisibleLocalScope(name))
				return makeScopedAttributeRef(cu, name, loc);
			if (ctx.currentModule() != null)
				return makeNameRef(name, loc);
			return new VariableRef(ctx.currentCFG(), loc, name);
		}

		// ── CASE 3: Module-level code
		if (ctx.isNameInVisibleLocalScope(name)) {
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

	public Expression makeScopedAttributeRef(
			CompilationUnit unit,
			String name,
			CodeLocation loc) {
		return new PythonScopedAttributeAccessRef(ctx.currentCFG(), loc, unit,
				new Global(loc, unit, name, false));
	}

	public Expression makeNameRef(
			String name,
			CodeLocation loc) {
		String modName = (ctx.currentModule() != null) ? ctx.currentModule().getName() : "__main__";
		String qualified = (ctx.imports() != null) ? ctx.imports().get(name) : null;
		return new PyNameRef(ctx.currentCFG(), loc, name, modName, qualified);
	}

	// === binary-operator folding ===

	@FunctionalInterface
	public interface BinaryFactory {
		Expression make(
				PyCFG cfg,
				CodeLocation loc,
				Expression l,
				Expression r);
	}

	public <C extends ParserRuleContext> Expression foldBinaryOp(
			List<C> operands,
			Function<C, Expression> sub,
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

	// === literal / structural helpers ===

	public StringLiteral strip(
			CodeLocation location,
			String string) {
		PyCFG cfg = ctx.currentCFG();
		if (string.startsWith("'''") && string.endsWith("'''"))
			return new PyStringLiteral(cfg, location, string.substring(3, string.length() - 3), "'''");
		if (string.startsWith("\"\"\"") && string.endsWith("\"\"\""))
			return new PyStringLiteral(cfg, location, string.substring(3, string.length() - 3), "\"\"\"");
		if (string.startsWith("'") && string.endsWith("'"))
			return new PyStringLiteral(cfg, location, string.substring(1, string.length() - 1), "'");
		if (string.startsWith("\"") && string.endsWith("\""))
			return new PyStringLiteral(cfg, location, string.substring(1, string.length() - 1), "\"");
		return new PyStringLiteral(cfg, location, string, "\"");
	}

	public Boolean isADict(
			DictorsetmakerContext pctx) {
		return pctx == null || pctx.test().size() == 2 * pctx.COLON().size();
	}

	public List<Expression> convertAssignmentsToByNameParameters(
			List<Expression> pars) {
		List<Expression> converted = new java.util.ArrayList<>(pars.size());
		for (Expression e : pars)
			if (!(e instanceof Assignment))
				converted.add(e);
			else
				converted.add(new NamedParameterExpression(e.getCFG(), e.getLocation(),
						((Assignment) e).getLeft().toString(), ((Assignment) e).getRight()));
		return converted;
	}

	public static String transformToCode(
			List<String> codeList) {
		return String.join("\n", codeList) + "\n";
	}

	// === CFG descriptor builders ===

	public CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		return new CodeMemberDescriptor(loc, ctx.currentModule(), false,
				ParserContext.INSTRUMENTED_MAIN_FUNCTION_NAME, new PyParameter[] {});
	}

	public CodeMemberDescriptor buildInitModuleCFGDescriptor(
			SourceCodeLocation loc) {
		return new CodeMemberDescriptor(loc, ctx.currentModule(), false, "$init", new PyParameter[] {});
	}

	public CodeMemberDescriptor buildInitClassCFGDescriptor(
			CodeLocation loc) {
		return new CodeMemberDescriptor(loc, ctx.currentUnit(), false, "$init", new PyParameter[] {});
	}

	// === CFG finalisation ===

	public void addRetNodesToCurrentCFG() {
		PyCFG currentCFG = ctx.currentCFG();
		if (currentCFG.getNodesCount() == 0) {
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

			// Every path already ends with an explicit stopping statement
			// (e.g. Return) — no need for a synthetic Ret.
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

		// Every non-throwing instruction without a follower is the method's
		// natural exit.
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

	public ModuleUnit currentModuleOrNull() {
		return ctx.currentModule();
	}
}
