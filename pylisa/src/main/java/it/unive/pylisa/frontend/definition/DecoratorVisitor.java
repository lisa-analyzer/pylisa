package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratedContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorsContext;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Triple;

/**
 * Handles Python decorator productions ({@code decorated}, {@code decorator},
 * {@code decorators}). Extracted from {@link DefinitionVisitor} in Chunk 5.
 */
public final class DecoratorVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public DecoratorVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Object visitDecorated(
			DecoratedContext pctx) {
		if (pctx.decorators().isEmpty()) {
			return support.unsupported(pctx, "Expecting a DecoratorsContext in DecoratedContext");
		}
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Expression result = null;
		Expression innerAssign = null;
		if (pctx.classdef() != null) {
			return support.unsupported(pctx, "Class Decorators are not supported yet");
		} else if (pctx.async_funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = ctx.def()
					.visitAsync_funcdef(pctx.async_funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(pctx.decorators(), func);
			} else {
				return support.unsupported(pctx, "Expecting a PyAssign while parsing async_funcDef");
			}
		} else if (pctx.funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = ctx.def()
					.visitFuncdef(pctx.funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(pctx.decorators(), func);
			} else {
				return support.unsupported(pctx, "Expecting a PyAssign while parsing funcDef");
			}
		} else {
			return support.unsupported(pctx, "Expecting {'def', 'class', 'async'} after decorators");
		}
		PyAssign pyAssign = new PyAssign(ctx.currentCFG(), support.getLocation(pctx), innerAssign, result);
		block.addNode(pyAssign);
		return Triple.of(pyAssign, block, pyAssign);
	}

	public FunctionApply visitDecorator(
			DecoratorContext pctx) {
		if (pctx.dotted_name() == null) {
			throw new UnsupportedOperationException("Expecting a Dotted_nameContext in a DecoratorContext.");
		}
		Expression result = ctx.stmt().visitDotted_name(pctx.dotted_name());
		/*
		 * If the result is a VariableRef, e.g. @f() -> VariableRef(f), it means
		 * we are in the current scope — no need to add a parameter in the
		 * function.
		 */

		if (result instanceof VariableRef) {
			List<Expression> params = new ArrayList<>();
			if (pctx.arglist() != null)
				for (ArgumentContext arg : pctx.arglist().argument())
					params.add(ctx.expr().visitArgument(arg));
			params = support.convertAssignmentsToByNameParameters(params);
			return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), result,
					params.toArray(Expression[]::new));
		}
		List<Expression> params = new ArrayList<>();
		String varName = pctx.dotted_name().children.get(0).getText();
		params.add(support.makeRef(varName, support.getLocation(pctx)));
		if (pctx.arglist() != null)
			for (ArgumentContext arg : pctx.arglist().argument())
				params.add(ctx.expr().visitArgument(arg));
		params = support.convertAssignmentsToByNameParameters(params);
		return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), result,
				params.toArray(Expression[]::new));
	}

	public Expression visitDecorators(
			DecoratorsContext pctx,
			Expression decoratedFunction) {
		Expression result = decoratedFunction;

		List<DecoratorContext> decorators = pctx.decorator();

		for (int i = decorators.size() - 1; i >= 0; i--) {
			DecoratorContext decorCtx = decorators.get(i);
			FunctionApply decorator = visitDecorator(decorCtx);

			if (result == null) {
				result = decorator;
			} else {
				// @f(args) → f(args)(func): decorator is a call, use it as
				// target (double-wrap)
				// @f → f(func): decorator IS the callable, extract target
				// (single-wrap)
				boolean hasExplicitParens = decorCtx.OPEN_PAREN() != null;
				Expression callTarget = hasExplicitParens
						? decorator
						: decorator.getSubExpressions()[0];
				result = new FunctionApply(
						ctx.currentCFG(),
						support.getLocation(pctx),
						callTarget,
						List.of(result).toArray(Expression[]::new));
			}
		}

		return result;
	}
}
