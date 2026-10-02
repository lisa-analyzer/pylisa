package it.unive.pylisa.frontend.statement;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Return;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.pylisa.antlr.Python3Parser.Break_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Continue_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Flow_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Raise_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Return_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_stmtContext;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.cfg.statement.PyRaise;
import it.unive.pylisa.cfg.type.PyExceptionType;
import it.unive.pylisa.frontend.BoundNames;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/**
 * Handles control-flow-terminating statements — return, break, continue, yield,
 * raise, and the {@code flow_stmt} wrapper. Extracted from
 * {@link StatementVisitor} in Chunk 4.
 */
public final class FlowControlVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public FlowControlVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Statement visitFlow_stmt(
			Flow_stmtContext pctx) {
		if (pctx.return_stmt() != null)
			return visitReturn_stmt(pctx.return_stmt());

		if (pctx.raise_stmt() != null)
			return visitRaise_stmt(pctx.raise_stmt());

		if (pctx.yield_stmt() != null) {
			// a generator's call returns a generator, not what its body
			// computes: the function no longer describes the program
			support.unsound(pctx, "yield treated as no-op");
			support.markGenerator();
			Yield_argContext yieldArg = pctx.yield_stmt().yield_expr().yield_arg();
			if (yieldArg == null) {
				return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
			}
			List<Expression> l = ctx.expr().extractExpressionsFromYieldArg(yieldArg);
			return new UnresolvedCall(
					ctx.currentCFG(),
					support.getLocation(pctx),
					CallType.STATIC,
					Program.PROGRAM_NAME,
					"yield from",
					LeftToRightEvaluation.INSTANCE,
					l.toArray(new Expression[0]));
		}

		if (pctx.continue_stmt() != null)
			return visitContinue_stmt(pctx.continue_stmt());

		if (pctx.break_stmt() != null)
			return visitBreak_stmt(pctx.break_stmt());

		return support.rejectUnsupported(pctx);
	}

	public Statement visitReturn_stmt(
			Return_stmtContext pctx) {
		if (pctx.testlist() == null)
			// a bare return returns None, or ends a generator
			return new Return(ctx.currentCFG(), support.getLocation(pctx),
					support.implicitReturnValue(support.getLocation(pctx)));
		if (pctx.testlist().test().size() == 1)
			return new Return(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitTest(pctx.testlist().test(0)));
		else {
			support.unsound(pctx, "multiple return values treated as first value");
			return new Return(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitTest(pctx.testlist().test(0)));
		}
	}

	public Statement visitBreak_stmt(
			Break_stmtContext pctx) {
		return new Break(ctx.currentCFG(), support.getLocation(pctx));
	}

	public Statement visitContinue_stmt(
			Continue_stmtContext pctx) {
		return new Continue(ctx.currentCFG(), support.getLocation(pctx));
	}

	public Object visitYield_stmt(
			Yield_stmtContext pctx) {
		// Mirror the yield-handling branch of visitFlow_stmt so that yield
		// reached through the dedicated grammar production (rather than via
		// flow_stmt) is also translated to a control-flow-preserving node.
		// Treating yield as a no-op is unsound for generator semantics, but
		// it keeps the surrounding CFG well-formed — necessary for any
		// project that uses async generators (notably FastAPI's
		// `async def lifespan(app): try: yield finally: ...` pattern).
		support.unsound(pctx, "yield treated as no-op");
		support.markGenerator();
		Yield_argContext yieldArg = pctx.yield_expr().yield_arg();
		if (yieldArg == null)
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		List<Expression> l = ctx.expr().extractExpressionsFromYieldArg(yieldArg);
		return new UnresolvedCall(
				ctx.currentCFG(),
				support.getLocation(pctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"yield from",
				LeftToRightEvaluation.INSTANCE,
				l.toArray(new Expression[0]));
	}

	/**
	 * Translates a {@code raise} statement. When the raised exception is built
	 * by, or is, a builtin exception class with a known type, the statement
	 * raises that type and evaluates only the arguments of the class; any other
	 * raised expression is evaluated as a whole and raises an exception of
	 * unknown type. The name is taken as the builtin class only when the file
	 * never binds it (see {@link BoundNames}).
	 *
	 * @param pctx the statement
	 *
	 * @return the translated statement
	 */
	public Statement visitRaise_stmt(
			Raise_stmtContext pctx) {
		List<Expression> evaluated = new ArrayList<>();
		PyExceptionType type = PyExceptionType.BASE_EXCEPTION;
		if (!pctx.test().isEmpty()) {
			Expression raised = ctx.expr().visitTest(pctx.test(0));
			PyExceptionType builtin = builtinException(raised, ctx.boundNames(pctx));
			if (builtin == null)
				evaluated.add(raised);
			else {
				type = builtin;
				if (raised instanceof PyCall call) {
					Expression[] sub = call.getSubExpressions();
					evaluated.addAll(Arrays.asList(sub).subList(1, sub.length));
				}
			}
			if (pctx.test().size() > 1)
				evaluated.add(ctx.expr().visitTest(pctx.test(1)));
		}
		return new PyRaise(ctx.currentCFG(), support.getLocation(pctx), type, evaluated.toArray(Expression[]::new));
	}

	private static PyExceptionType builtinException(
			Expression raised,
			BoundNames bound) {
		Expression named = raised instanceof PyCall call ? call.getSubExpressions()[0] : raised;
		return named instanceof PyNameRef name && !bound.mayBind(name.getName())
				? PyExceptionType.builtin(name.getName())
				: null;
	}
}
