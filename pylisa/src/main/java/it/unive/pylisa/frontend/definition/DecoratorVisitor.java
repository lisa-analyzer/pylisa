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
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.commons.lang3.tuple.Triple;

/**
 * Handles Python decorator productions ({@code decorated}, {@code decorator},
 * {@code decorators}). Extracted from {@link DefinitionVisitor} in Chunk 5.
 */
public final class DecoratorVisitor {

	private static final Logger LOG = LogManager.getLogger(DecoratorVisitor.class);

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
			// **Unsoundness:** we don't model class decorators. Some are inert
			// for our purposes ({@code @dataclass}, {@code @final},
			// {@code @runtime_checkable}, …); others have real semantic effect
			// — e.g. {@code @site.register_admin} registers the class with an
			// admin router, {@code @strawberry.type} marks it as a GraphQL
			// type, custom registry decorators install side-effecting hooks.
			// Each occurrence is marked as a known limitation of its function
			// (and logged with the decorator names), then the classdef is
			// visited as if undecorated to keep the rest of the analysis
			// reachable.
			List<DecoratorContext> decorators = pctx.decorators().decorator();
			String names = decorators.stream()
					.map(DecoratorVisitor::decoratorName)
					.collect(Collectors.joining(", "));
			LOG.warn("[PyLiSA] Skipping {} class decorator(s) at {}: [{}] — analysis "
					+ "will proceed as if the class were undecorated (UNSOUND for "
					+ "decorators with side effects, e.g. registry/handler installation)",
					decorators.size(), support.getLocation(pctx), names);
			support.limitation(pctx, "class decorator not applied");
			return ctx.def().visitClassdef(pctx.classdef());
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

	/**
	 * Best-effort string rendering of a decorator name, used purely for the
	 * unsoundness log emitted when we skip a class decorator. Returns the
	 * dotted name (e.g. {@code "site.register_admin"}) or {@code "<unknown>"}
	 * when the parse tree shape is unexpected.
	 */
	private static String decoratorName(
			DecoratorContext d) {
		if (d == null || d.dotted_name() == null || d.dotted_name().getText() == null)
			return "<unknown>";
		return d.dotted_name().getText();
	}

	public PyCall visitDecorator(
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
			return new PyCall(ctx.currentCFG(), support.getLocation(pctx), result,
					params.toArray(Expression[]::new));
		}
		List<Expression> params = new ArrayList<>();
		String varName = pctx.dotted_name().children.get(0).getText();
		params.add(support.makeRef(varName, support.getLocation(pctx)));
		if (pctx.arglist() != null)
			for (ArgumentContext arg : pctx.arglist().argument())
				params.add(ctx.expr().visitArgument(arg));
		return new PyCall(ctx.currentCFG(), support.getLocation(pctx), result,
				params.toArray(Expression[]::new));
	}

	public Expression visitDecorators(
			DecoratorsContext pctx,
			Expression decoratedFunction) {
		Expression result = decoratedFunction;

		List<DecoratorContext> decorators = pctx.decorator();

		for (int i = decorators.size() - 1; i >= 0; i--) {
			DecoratorContext decorCtx = decorators.get(i);
			PyCall decorator = visitDecorator(decorCtx);

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
				// Tag decorator-position calls so that, when the call target's
				// runtime type is unresolved (e.g. an external decorator with
				// no library spec like @slowapi.Limiter.limit(...)),
				// PyCall's semantics fall back to passing the inner
				// argument through unchanged. This preserves the decorated
				// function's PyFunctionType so an outer route decorator can
				// still resolve the handler.
				result = new PyCall(
						ctx.currentCFG(),
						support.getLocation(pctx),
						callTarget,
						List.of(result).toArray(Expression[]::new),
						false,
						true);
			}
		}

		return result;
	}
}
