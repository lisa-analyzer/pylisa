package it.unive.pylisa.scoping;

import static it.unive.pylisa.PyParsingUtils.getLocation;

import it.unive.pylisa.ParsingException;
import it.unive.pylisa.antlr.PythonParser.AssignmentContext;
import it.unive.pylisa.antlr.PythonParser.Assignment_expressionContext;
import it.unive.pylisa.antlr.PythonParser.AtomContext;
import it.unive.pylisa.antlr.PythonParser.AttrContext;
import it.unive.pylisa.antlr.PythonParser.Class_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.Default_assignmentContext;
import it.unive.pylisa.antlr.PythonParser.Del_t_atomContext;
import it.unive.pylisa.antlr.PythonParser.DictcompContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_nameContext;
import it.unive.pylisa.antlr.PythonParser.Except_blockContext;
import it.unive.pylisa.antlr.PythonParser.Except_star_blockContext;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParser.For_if_clauseContext;
import it.unive.pylisa.antlr.PythonParser.For_if_clausesContext;
import it.unive.pylisa.antlr.PythonParser.Function_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.GenexpContext;
import it.unive.pylisa.antlr.PythonParser.Global_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Import_fromContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_nameContext;
import it.unive.pylisa.antlr.PythonParser.Lambda_paramContext;
import it.unive.pylisa.antlr.PythonParser.LambdefContext;
import it.unive.pylisa.antlr.PythonParser.ListcompContext;
import it.unive.pylisa.antlr.PythonParser.NameContext;
import it.unive.pylisa.antlr.PythonParser.Name_or_attrContext;
import it.unive.pylisa.antlr.PythonParser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.PythonParser.ParamContext;
import it.unive.pylisa.antlr.PythonParser.Param_star_annotationContext;
import it.unive.pylisa.antlr.PythonParser.Pattern_capture_targetContext;
import it.unive.pylisa.antlr.PythonParser.SetcompContext;
import it.unive.pylisa.antlr.PythonParser.Single_targetContext;
import it.unive.pylisa.antlr.PythonParser.Star_atomContext;
import it.unive.pylisa.antlr.PythonParser.Star_patternContext;
import it.unive.pylisa.antlr.PythonParser.Type_aliasContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.scoping.PyScope.ScopeKind;
import java.util.HashSet;
import java.util.Set;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ParseTree;

/**
 * A visitor of the parse tree of a Python module that builds its
 * {@link Symtable}, following the rules of CPython's symbol tables. The visitor
 * first collects, for each {@link PyScope}, the names that are bound, read, and
 * declared {@code global} or {@code nonlocal} there, and then it resolves the
 * {@link NameKind} of each of them. The scoping errors that Python reports as
 * {@code SyntaxError}s (e.g. a {@code nonlocal} declaration of a name that is
 * not bound by any enclosing function) are reported as
 * {@link ParsingException}s.<br>
 * <br>
 * Comprehensions and generator expressions are evaluated inline by pylisa (as
 * CPython does for comprehensions since PEP 709): only their iteration
 * variables belong to the scope of the comprehension, while the other names
 * read inside them are attributed to the closest enclosing scope that is not a
 * comprehension, and the names bound by assignment expressions
 * ({@code x := ...}) are bound there.
 */
public class SymtableBuilder extends PythonParserBaseVisitor<Void> {

	private final String filePath;

	private final Symtable symtable;

	private PyScope current;

	private SymtableBuilder(
			String filePath,
			String module,
			File_inputContext ctx) {
		this.filePath = filePath;
		this.current = new PyScope(ScopeKind.MODULE, module, module, null, ctx);
		this.symtable = new Symtable(current);
	}

	/**
	 * Builds the symbol table of a module.
	 *
	 * @param filePath the path of the file of the module
	 * @param module   the name of the module
	 * @param ctx      the parse tree of the module
	 *
	 * @return the symbol table
	 *
	 * @throws ParsingException if the module contains scoping errors that
	 *                              Python reports as {@code SyntaxError}s
	 */
	public static Symtable build(
			String filePath,
			String module,
			File_inputContext ctx) {
		SymtableBuilder builder = new SymtableBuilder(filePath, module, ctx);
		builder.visitChildren(ctx);
		builder.resolve(builder.symtable.getModuleScope(), new HashSet<>());
		return builder.symtable;
	}

	private ParsingException syntaxError(
			String message,
			ParserRuleContext ctx) {
		return new ParsingException("syntax error", ParsingException.Type.MALFORMED_SOURCE, message,
				getLocation(filePath, ctx));
	}

	private PyScope enter(
			ScopeKind kind,
			String name,
			ParserRuleContext node) {
		PyScope scope = new PyScope(kind, name, current.getModule(), current, node);
		symtable.register(scope);
		return scope;
	}

	// ---------------------------------------------------------------------
	// collection of names
	// ---------------------------------------------------------------------

	// the closest scope, starting from the current one, that is not a
	// comprehension, or the comprehension binding the given name (if any)
	private PyScope readScope(
			String name) {
		PyScope scope = current;
		while (scope.getKind() == ScopeKind.COMPREHENSION && !scope.isBound(name))
			scope = scope.getParent();
		return scope;
	}

	private void read(
			ParserRuleContext name) {
		PyScope scope = readScope(name.getText());
		scope.addRead(name.getText());
		symtable.addOccurrence(name, scope);
	}

	private void bind(
			ParserRuleContext name) {
		bind(name, current);
	}

	private void bind(
			ParserRuleContext name,
			PyScope scope) {
		scope.addBound(name.getText());
		symtable.addOccurrence(name, scope);
	}

	@Override
	public Void visitAtom(
			AtomContext ctx) {
		if (ctx.name() != null)
			read(ctx.name());
		else
			visitChildren(ctx);
		return null;
	}

	@Override
	public Void visitAttr(
			AttrContext ctx) {
		// e.g. Color.RED in a value pattern: Color is read
		read(ctx.name(0));
		return null;
	}

	@Override
	public Void visitName_or_attr(
			Name_or_attrContext ctx) {
		// e.g. Point in a class pattern: Point is read
		read(ctx.name(0));
		return null;
	}

	@Override
	public Void visitStar_atom(
			Star_atomContext ctx) {
		if (ctx.name() != null)
			bind(ctx.name());
		else
			visitChildren(ctx);
		return null;
	}

	@Override
	public Void visitSingle_target(
			Single_targetContext ctx) {
		if (ctx.name() != null)
			bind(ctx.name());
		else
			visitChildren(ctx);
		return null;
	}

	@Override
	public Void visitDel_t_atom(
			Del_t_atomContext ctx) {
		// deleting a name makes it local, as binding it
		if (ctx.name() != null)
			bind(ctx.name());
		else
			visitChildren(ctx);
		return null;
	}

	@Override
	public Void visitAssignment(
			AssignmentContext ctx) {
		if (ctx.expression() == null)
			return visitChildren(ctx);

		// an annotated assignment: the annotation is not evaluated inside
		// functions, and it is thus not visited
		if (ctx.name() != null)
			bind(ctx.name());
		else if (ctx.single_target() != null)
			visit(ctx.single_target());
		else
			visit(ctx.single_subscript_attribute_target());
		if (ctx.annotated_rhs() != null)
			visit(ctx.annotated_rhs());
		return null;
	}

	@Override
	public Void visitAssignment_expression(
			Assignment_expressionContext ctx) {
		visit(ctx.expression());
		// the name is bound in the closest scope that is not a comprehension
		PyScope scope = current;
		while (scope.getKind() == ScopeKind.COMPREHENSION) {
			if (scope.isBound(ctx.name().getText()))
				throw syntaxError("assignment expression cannot rebind comprehension iteration variable '"
						+ ctx.name().getText() + "'", ctx);
			scope = scope.getParent();
		}
		bind(ctx.name(), scope);
		return null;
	}

	@Override
	public Void visitImport_name(
			Import_nameContext ctx) {
		for (Dotted_as_nameContext imported : ctx.dotted_as_names().dotted_as_name())
			if (imported.name() != null)
				// import a.b as c binds c
				bind(imported.name());
			else {
				// import a.b binds a
				Dotted_nameContext dotted = imported.dotted_name();
				while (dotted.dotted_name() != null)
					dotted = dotted.dotted_name();
				bind(dotted.name());
			}
		return null;
	}

	@Override
	public Void visitImport_from(
			Import_fromContext ctx) {
		if (ctx.import_from_targets().STAR() != null) {
			if (current.getKind() != ScopeKind.MODULE)
				throw syntaxError("import * only allowed at module level", ctx);
			current.setStarImport();
			return null;
		}

		for (Import_from_as_nameContext imported : ctx.import_from_targets().import_from_as_names()
				.import_from_as_name())
			// from m import a as b binds b, from m import a binds a
			bind(imported.name(imported.name().size() - 1));
		return null;
	}

	@Override
	public Void visitExcept_block(
			Except_blockContext ctx) {
		if (ctx.expression() != null)
			visit(ctx.expression());
		if (ctx.expressions() != null)
			visit(ctx.expressions());
		if (ctx.name() != null)
			bind(ctx.name());
		visit(ctx.block());
		return null;
	}

	@Override
	public Void visitExcept_star_block(
			Except_star_blockContext ctx) {
		if (ctx.expression() != null)
			visit(ctx.expression());
		if (ctx.expressions() != null)
			visit(ctx.expressions());
		if (ctx.name() != null)
			bind(ctx.name());
		visit(ctx.block());
		return null;
	}

	@Override
	public Void visitPattern_capture_target(
			Pattern_capture_targetContext ctx) {
		bind(ctx.name_except_underscore());
		return null;
	}

	@Override
	public Void visitStar_pattern(
			Star_patternContext ctx) {
		// *_ does not bind anything
		if (!ctx.name().getText().equals("_"))
			bind(ctx.name());
		return null;
	}

	@Override
	public Void visitType_alias(
			Type_aliasContext ctx) {
		bind(ctx.name());
		visit(ctx.expression());
		return null;
	}

	@Override
	public Void visitGlobal_stmt(
			Global_stmtContext ctx) {
		for (NameContext name : ctx.name()) {
			String n = name.getText();
			if (current.isParameter(n))
				throw syntaxError("name '" + n + "' is parameter and global", name);
			if (current.isNonlocal(n))
				throw syntaxError("name '" + n + "' is nonlocal and global", name);
			if (current.isBound(n))
				throw syntaxError("name '" + n + "' is assigned to before global declaration", name);
			if (current.isRead(n))
				throw syntaxError("name '" + n + "' is used prior to global declaration", name);
			current.addGlobal(n);
			symtable.addOccurrence(name, current);
		}
		return null;
	}

	@Override
	public Void visitNonlocal_stmt(
			Nonlocal_stmtContext ctx) {
		if (current.getKind() == ScopeKind.MODULE)
			throw syntaxError("nonlocal declaration not allowed at module level", ctx);
		for (NameContext name : ctx.name()) {
			String n = name.getText();
			if (current.isParameter(n))
				throw syntaxError("name '" + n + "' is parameter and nonlocal", name);
			if (current.isGlobal(n))
				throw syntaxError("name '" + n + "' is nonlocal and global", name);
			if (current.isBound(n))
				throw syntaxError("name '" + n + "' is assigned to before nonlocal declaration", name);
			if (current.isRead(n))
				throw syntaxError("name '" + n + "' is used prior to nonlocal declaration", name);
			current.addNonlocal(n);
			symtable.addOccurrence(name, current);
		}
		return null;
	}

	// ---------------------------------------------------------------------
	// scopes
	// ---------------------------------------------------------------------

	@Override
	public Void visitFunction_def_raw(
			Function_def_rawContext ctx) {
		// the decorators are visited by visitFunction_def, in the enclosing
		// scope, as the name of the function, the default values of the
		// parameters, and the annotations
		bind(ctx.name());
		PyScope function = enter(ScopeKind.FUNCTION, ctx.name().getText(), ctx);
		if (ctx.params() != null)
			parameters(ctx.params(), function);
		if (ctx.expression() != null)
			// the return annotation
			visit(ctx.expression());

		PyScope enclosing = current;
		current = function;
		visit(ctx.block());
		current = enclosing;
		return null;
	}

	@Override
	public Void visitLambdef(
			LambdefContext ctx) {
		PyScope lambda = enter(ScopeKind.FUNCTION, "<lambda>", ctx);
		if (ctx.lambda_params() != null)
			parameters(ctx.lambda_params(), lambda);

		PyScope enclosing = current;
		current = lambda;
		visit(ctx.expression());
		current = enclosing;
		return null;
	}

	// collects the parameters of the function into its scope, visiting their
	// default values and annotations in the current (enclosing) scope
	private void parameters(
			ParseTree node,
			PyScope function) {
		if (node instanceof ParamContext) {
			parameter(((ParamContext) node).name(), function);
			if (((ParamContext) node).annotation() != null)
				visit(((ParamContext) node).annotation());
		} else if (node instanceof Param_star_annotationContext) {
			parameter(((Param_star_annotationContext) node).name(), function);
			visit(((Param_star_annotationContext) node).star_annotation());
		} else if (node instanceof Lambda_paramContext)
			parameter(((Lambda_paramContext) node).name(), function);
		else if (node instanceof Default_assignmentContext)
			visit(node);
		else
			for (int i = 0; i < node.getChildCount(); i++)
				parameters(node.getChild(i), function);
	}

	private void parameter(
			NameContext name,
			PyScope function) {
		if (function.isParameter(name.getText()))
			throw syntaxError("duplicate argument '" + name.getText() + "' in function definition", name);
		function.addParameter(name.getText());
		symtable.addOccurrence(name, function);
	}

	@Override
	public Void visitClass_def_raw(
			Class_def_rawContext ctx) {
		// the decorators are visited by visitClass_def, in the enclosing
		// scope, as the name of the class and its bases
		bind(ctx.name());
		if (ctx.arguments() != null)
			visit(ctx.arguments());
		PyScope cls = enter(ScopeKind.CLASS, ctx.name().getText(), ctx);

		PyScope enclosing = current;
		current = cls;
		visit(ctx.block());
		current = enclosing;
		return null;
	}

	@Override
	public Void visitListcomp(
			ListcompContext ctx) {
		comprehension(ctx, "<listcomp>", ctx.for_if_clauses(), ctx.named_expression());
		return null;
	}

	@Override
	public Void visitSetcomp(
			SetcompContext ctx) {
		comprehension(ctx, "<setcomp>", ctx.for_if_clauses(), ctx.named_expression());
		return null;
	}

	@Override
	public Void visitDictcomp(
			DictcompContext ctx) {
		comprehension(ctx, "<dictcomp>", ctx.for_if_clauses(), ctx.kvpair());
		return null;
	}

	@Override
	public Void visitGenexp(
			GenexpContext ctx) {
		comprehension(ctx, "<genexpr>", ctx.for_if_clauses(),
				ctx.assignment_expression() != null ? ctx.assignment_expression() : ctx.expression());
		return null;
	}

	private void comprehension(
			ParserRuleContext ctx,
			String name,
			For_if_clausesContext clauses,
			ParserRuleContext element) {
		// the first iterable is evaluated in the enclosing scope
		visit(clauses.for_if_clause(0).disjunction(0));
		PyScope comprehension = enter(ScopeKind.COMPREHENSION, name, ctx);

		PyScope enclosing = current;
		current = comprehension;
		boolean first = true;
		for (For_if_clauseContext clause : clauses.for_if_clause()) {
			if (!first)
				visit(clause.disjunction(0));
			first = false;
			visit(clause.star_targets());
			for (int i = 1; i < clause.disjunction().size(); i++)
				visit(clause.disjunction(i));
		}
		visit(element);
		current = enclosing;
	}

	// ---------------------------------------------------------------------
	// resolution
	// ---------------------------------------------------------------------

	/**
	 * Resolves the kinds of the names of the given scope and of the scopes
	 * nested into it, returning the names that the scope refers to as free
	 * variables (including the ones of nested scopes that are not bound by it,
	 * and that are thus bound by some enclosing function).
	 *
	 * @param scope     the scope to resolve
	 * @param enclosing the names bound by the enclosing functions, that are
	 *                      visible from the scope
	 *
	 * @return the free variables of the scope
	 */
	private Set<String> resolve(
			PyScope scope,
			Set<String> enclosing) {
		for (String name : scope.allNames())
			if (scope.isGlobal(name))
				scope.setKind(name, NameKind.GLOBAL_EXPLICIT);
			else if (scope.isNonlocal(name)) {
				if (!enclosing.contains(name))
					throw syntaxError("no binding for nonlocal '" + name + "' found", scope.getNode());
				scope.setKind(name, NameKind.FREE);
			} else if (scope.getKind() == ScopeKind.MODULE)
				scope.setKind(name, NameKind.GLOBAL_IMPLICIT);
			else if (scope.binds(name))
				scope.setKind(name, NameKind.LOCAL);
			else if (enclosing.contains(name))
				scope.setKind(name, NameKind.FREE);
			else
				scope.setKind(name, NameKind.GLOBAL_IMPLICIT);

		// the names visible from nested scopes: the ones of the enclosing
		// functions, plus the local and free ones of this function (class
		// bodies are not visible from nested scopes)
		Set<String> visible = new HashSet<>();
		if (scope.getKind() != ScopeKind.MODULE) {
			visible.addAll(enclosing);
			for (String name : scope.allNames()) {
				NameKind kind = scope.getResolvedKind(name);
				if (kind == NameKind.GLOBAL_EXPLICIT)
					visible.remove(name);
				else if (scope.isFunctionLike() && (kind == NameKind.LOCAL || kind == NameKind.FREE))
					visible.add(name);
			}
		}

		Set<String> free = new HashSet<>(scope.getFreeVariables());
		for (PyScope child : scope.getChildren())
			for (String name : resolve(child, visible)) {
				NameKind kind = scope.getResolvedKind(name);
				if (scope.isFunctionLike() && kind == NameKind.LOCAL)
					// captured by the nested scope
					scope.setKind(name, NameKind.CELL);
				else if (scope.isFunctionLike() && kind == NameKind.CELL)
					continue;
				else {
					// bound by an enclosing function: it is passed through
					// this scope to the nested one
					if (kind == null)
						scope.setKind(name, NameKind.FREE);
					free.add(name);
				}
			}
		return free;
	}
}
