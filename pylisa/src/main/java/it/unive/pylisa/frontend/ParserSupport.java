package it.unive.pylisa.frontend;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.values.StringAnnotationValue;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Return;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.Atom_exprContext;
import it.unive.pylisa.antlr.Python3Parser.TrailerContext;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.expression.literal.PyUnknownLiteral;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PySourceCodeLocation;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;
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

	/**
	 * The functions translated so far whose body contains {@code yield}: their
	 * calls return a generator, not what their body computes.
	 */
	private final Set<CFG> generators = new HashSet<>();


	/**
	 * The annotation of a function (or module body) part of which the
	 * frontend translated unsoundly: its analysis may miss executions of the
	 * program.
	 */
	public static final String UNSOUND_TRANSLATION = "pylisa.unsound-translation";

	/**
	 * The annotation of a function in which a construct is translated in a
	 * known unsound way that does not weaken the analysis: results near it
	 * may be wrong in either direction, and readers list it.
	 */
	public static final String KNOWN_LIMITATION = "pylisa.known-limitation";

	/**
	 * The member of a {@link #KNOWN_LIMITATION} or
	 * {@link #UNSOUND_TRANSLATION} annotation naming the construct.
	 */
	public static final String CONSTRUCT = "construct";

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

	/**
	 * Yields the location of a construct.
	 *
	 * @param pctx the construct
	 *
	 * @return the location
	 */
	public SourceCodeLocation getLocation(
			ParserRuleContext pctx) {
		return getLocation(pctx, pctx.getStart());
	}

	/**
	 * Yields the location of a construct that is part of a larger one, such as
	 * the argument list of a call: its line and end column are those of the
	 * part, which tell constructs apart, and its start is the given token,
	 * the start of the whole construct, where reports point.
	 *
	 * @param pctx  the part
	 * @param start the first token of the whole construct
	 *
	 * @return the location
	 */
	public SourceCodeLocation getLocation(
			ParserRuleContext pctx,
			Token start) {
		// the source file comes from the token's stream, so that a sub-module
		// translated while the entry file is being translated gets its own
		// path
		String source = ctx.filePath();
		if (pctx != null && pctx.getStart() != null) {
			org.antlr.v4.runtime.CharStream cs = pctx.getStart().getInputStream();
			if (cs != null) {
				String csName = cs.getSourceName();
				if (csName != null && !csName.isEmpty() && !"<unknown>".equals(csName))
					source = csName;
			}
		}
		return new PySourceCodeLocation(source, getLine(pctx), getCol(pctx), start.getLine(),
				start.getCharPositionInLine());
	}

	/**
	 * Like {@link #getLocation(ParserRuleContext)} but anchored at the
	 * {@code stop} token of the rule rather than the {@code start} — useful
	 * when a single rule contributes two synthetic statements (e.g. an
	 * entry and an exit {@link it.unive.lisa.program.cfg.statement.NoOp})
	 * that need distinct locations to avoid being de-duplicated by
	 * {@code Statement.equals} (which compares only class + location).
	 */
	public SourceCodeLocation getStopLocation(
			ParserRuleContext pctx) {
		String source = ctx.filePath();
		if (pctx != null && pctx.getStop() != null) {
			org.antlr.v4.runtime.CharStream cs = pctx.getStop().getInputStream();
			if (cs != null) {
				String csName = cs.getSourceName();
				if (csName != null && !csName.isEmpty() && !"<unknown>".equals(csName))
					source = csName;
			}
		}
		int line = pctx != null && pctx.getStop() != null ? pctx.getStop().getLine() : -1;
		int col = pctx != null && pctx.getStop() != null
				? pctx.getStop().getCharPositionInLine()
				: -1;
		// Bump col by 1 to differ from any token-aligned location at the
		// stop position; SourceCodeLocation rejects -1, but any other value
		// is fine.
		return new PySourceCodeLocation(source, Math.max(line, 0), Math.max(col, 0) + 1, Math.max(line, 0),
				Math.max(col, 0));
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

	/**
	 * Marks the function being translated as no longer describing the
	 * program faithfully: analyses reading its results weaken them. The mark
	 * names the construct by the description.
	 *
	 * @param pctx        the construct translated unsoundly
	 * @param description what the translation drops or approximates
	 */
	public void unsound(
			ParserRuleContext pctx,
			String description) {
		mark(pctx, new Annotation(UNSOUND_TRANSLATION,
				List.of(new AnnotationMember(CONSTRUCT, new StringAnnotationValue(description)))),
				DiagnosticReporter.Severity.UNSOUND, description, "unsound translation");
	}

	/**
	 * Marks the function being translated as containing a construct that the
	 * frontend translates in a known unsound way without weakening the
	 * analysis, such as a call whose arguments contain another call (the
	 * inner call is evaluated more than once).
	 *
	 * @param pctx      the construct
	 * @param construct the name of the construct, as readers report it
	 */
	public void limitation(
			ParserRuleContext pctx,
			String construct) {
		mark(pctx, new Annotation(KNOWN_LIMITATION,
				List.of(new AnnotationMember(CONSTRUCT, new StringAnnotationValue(construct)))),
				DiagnosticReporter.Severity.LIMITATION, construct, "known frontend limitation");
	}

	private void mark(
			ParserRuleContext pctx,
			Annotation annotation,
			DiagnosticReporter.Severity severity,
			String description,
			String kind) {
		SourceCodeLocation loc = getLocation(pctx);
		// a mark outside any function would be lost, and readers would take
		// the translation as faithful
		if (ctx.currentCFG() == null)
			throw new IllegalStateException(description + " (" + kind + ") at " + loc
					+ " outside any function: the mark would be lost");
		ctx.currentCFG().getDescriptor().addAnnotation(annotation);
		ctx.reporter().report(severity, loc, featureLabel(description, pctx),
				description + " (" + kind + ") at line " + getLine(pctx) + " of " + ctx.filePath());
	}

	/**
	 * Yields the subscript an assignment target writes to, when the target is
	 * a subscript: the last trailer of {@code receiver[key]}, reached through
	 * the single-child nodes the grammar wraps a plain expression in.
	 *
	 * @param target the target of the assignment
	 *
	 * @return the subscript trailer, or {@code null} if the target is not a
	 *             subscript
	 */
	public static TrailerContext writtenSubscript(
			ParseTree target) {
		ParseTree node = target;
		while (!(node instanceof Atom_exprContext) && node.getChildCount() == 1)
			node = node.getChild(0);
		if (!(node instanceof Atom_exprContext expression) || expression.trailer().isEmpty())
			return null;
		TrailerContext last = expression.trailer(expression.trailer().size() - 1);
		return last.OPEN_BRACK() != null ? last : null;
	}

	/**
	 * Yields whether an expression contains a call to anything but
	 * {@code super}, whose repeated evaluation has no effect.
	 *
	 * @param expression the expression
	 *
	 * @return {@code true} if it contains such a call
	 */
	public static boolean containsCall(
			Expression expression) {
		if (expression instanceof PyCall call && !isSuperCall(call))
			return true;
		if (expression instanceof NaryExpression nary)
			for (Expression sub : nary.getSubExpressions())
				if (containsCall(sub))
					return true;
		return false;
	}

	private static boolean isSuperCall(
			PyCall call) {
		return call.getSubExpressions().length > 0
				&& call.getSubExpressions()[0] instanceof PyNameRef name
				&& "super".equals(name.getName());
	}

	/**
	 * Collapses a whole-method dead-stub visit override to a one-liner.
	 * Delegates to
	 * {@link UnsupportedGrammarFeatures#reject(ParserContext, ParserRuleContext, ParserSupport)}
	 * — reports an {@code UNSUPPORTED} diagnostic and throws. The generic
	 * return type lets callers write
	 * {@code return support.rejectUnsupported(ctx);} from any visit method
	 * irrespective of its declared return type; control never returns.
	 */
	public <T> T rejectUnsupported(
			ParserRuleContext pctx) {
		return UnsupportedGrammarFeatures.reject(this.ctx, pctx, this);
	}

	/**
	 * Overload for feature-gap branches inside larger visit methods: skips the
	 * registry lookup and uses {@code label} directly.
	 */
	public <T> T rejectUnsupported(
			ParserRuleContext pctx,
			String label) {
		return UnsupportedGrammarFeatures.reject(this.ctx, pctx, this, label);
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

	/**
	 * Translates a string literal token, prefix included. Formatted strings
	 * ({@code f"..."}) and bytes literals ({@code b"..."}) are not evaluated
	 * and translate to unknown values; raw and unicode prefixes are dropped and
	 * the literal translates to its contents.
	 *
	 * @param location the location of the literal
	 * @param token    the literal as written in the source
	 *
	 * @return the literal
	 */
	public Expression strip(
			CodeLocation location,
			String token) {
		int quote = 0;
		while (quote < token.length() && Character.isLetter(token.charAt(quote)))
			quote++;
		String prefix = token.substring(0, quote).toLowerCase();
		if (prefix.contains("f"))
			return new PyUnknownLiteral(ctx.currentCFG(), location, token, StringType.INSTANCE);
		if (prefix.contains("b"))
			return new PyUnknownLiteral(ctx.currentCFG(), location, token, Untyped.INSTANCE);
		return stripQuotes(location, token.substring(quote));
	}

	private StringLiteral stripQuotes(
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

	/**
	 * Records that the function being translated contains {@code yield}.
	 */
	public void markGenerator() {
		generators.add(ctx.currentCFG());
	}

	/**
	 * Yields the value a {@code return} without a value gives in the function
	 * being translated: {@code None}, or an unknown value for a generator, whose
	 * call returns a generator object.
	 *
	 * @param location the location of the value
	 *
	 * @return the value
	 */
	public Expression implicitReturnValue(
			CodeLocation location) {
		if (generators.contains(ctx.currentCFG()))
			return new PyUnknownLiteral(ctx.currentCFG(), location, "generator", Untyped.INSTANCE);
		return new PyNoneLiteral(ctx.currentCFG(), location);
	}

	/**
	 * Makes every point where the body of the function being translated ends
	 * without a {@code return} return {@code None} (an unknown value for a
	 * generator), as Python does: every node
	 * without followers that does not stop the execution (statements after a
	 * {@code return}, which no execution reaches, included) flows into one
	 * {@code return None} at the given location.
	 *
	 * @param location the location of the implicit return, distinct from the
	 *                     locations of the statements of the body
	 */
	public void returnNoneAtNaturalExits(
			SourceCodeLocation location) {
		PyCFG currentCFG = ctx.currentCFG();
		Collection<Statement> exits = new LinkedList<>();
		for (Statement st : currentCFG.getNodes())
			if (!st.stopsExecution() && currentCFG.followersOf(st).isEmpty())
				exits.add(st);
		if (exits.isEmpty())
			return;
		Return implicit = new Return(currentCFG, location, implicitReturnValue(location));
		currentCFG.addNode(implicit);
		for (Statement exit : exits)
			currentCFG.addEdge(new SequentialEdge(exit, implicit));
	}

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
