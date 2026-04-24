package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.antlr.Python3Parser.Async_funcdefContext;
import it.unive.pylisa.antlr.Python3Parser.ClassdefContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratedContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorsContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.StarargsContext;
import it.unive.pylisa.antlr.Python3Parser.TfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarkwContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Triple;

/**
 * Root dispatcher for Python definition parsing. Delegates each grammar rule to
 * one of four category sub-visitors introduced in Chunk 5:
 * {@link ParameterVisitor}, {@link DecoratorVisitor},
 * {@link FunctionDefinitionVisitor}, and {@link ClassDefinitionVisitor}.
 * Cross-category recursion inside those sub-visitors routes back through
 * {@link ParserContext#def()}, so every rule resolves via this dispatcher.
 */
public final class DefinitionVisitor extends Python3ParserBaseVisitor<Object> {

	private final ParameterVisitor params;
	private final DecoratorVisitor decorators;
	private final FunctionDefinitionVisitor functions;
	private final ClassDefinitionVisitor classes;

	public DefinitionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		Objects.requireNonNull(ctx);
		Objects.requireNonNull(support);
		this.params = new ParameterVisitor(ctx, support);
		this.decorators = new DecoratorVisitor(ctx, support);
		this.functions = new FunctionDefinitionVisitor(ctx, support);
		this.classes = new ClassDefinitionVisitor(ctx, support);
	}

	// === decorators ===

	@Override
	public Object visitDecorated(
			DecoratedContext c) {
		return decorators.visitDecorated(c);
	}

	@Override
	public FunctionApply visitDecorator(
			DecoratorContext c) {
		return decorators.visitDecorator(c);
	}

	public Expression visitDecorators(
			DecoratorsContext c,
			Expression decoratedFunction) {
		return decorators.visitDecorators(c, decoratedFunction);
	}

	// === function definitions ===

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFuncdef(
			FuncdefContext c) {
		return functions.visitFuncdef(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitAsync_funcdef(
			Async_funcdefContext c) {
		return functions.visitAsync_funcdef(c);
	}

	public CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl,
			Unit unit) {
		return functions.buildCFGDescriptor(funcDecl, unit);
	}

	// === class definitions ===

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitClassdef(
			ClassdefContext c) {
		return classes.visitClassdef(c);
	}

	// === parameters ===

	@Override
	public PyParameter[] visitParameters(
			ParametersContext c) {
		return params.visitParameters(c);
	}

	@Override
	public PyParameter[] visitTypedargslist(
			TypedargslistContext c) {
		return params.visitTypedargslist(c);
	}

	@Override
	public PyParameter[] visitStarargs(
			StarargsContext c) {
		return params.visitStarargs(c);
	}

	@Override
	public PyParameter visitVarkw(
			VarkwContext c) {
		return params.visitVarkw(c);
	}

	@Override
	public PyParameter visitTypedarg(
			TypedargContext c) {
		return params.visitTypedarg(c);
	}

	@Override
	public PyParameter visitTfpdef(
			TfpdefContext c) {
		return params.visitTfpdef(c);
	}

	@Override
	public Object visitVarargslist(
			VarargslistContext c) {
		return params.visitVarargslist(c);
	}

	@Override
	public String visitVfpdef(
			VfpdefContext c) {
		return params.visitVfpdef(c);
	}
}
