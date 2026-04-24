package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.Atom_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Subscript_Context;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.TrailerContext;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles {@code atom_expr} trailer chaining: attribute access, subscripts,
 * function calls, and the Python 3 zero-arg {@code super()} synthesis.
 * Extracted from {@link ExpressionVisitor} in Chunk 3. Cross-visitor recursion
 * (e.g. resolving the base {@code atom} or sub-expressions inside arguments)
 * goes through {@code ctx.expr()}.
 */
public final class AccessVisitor {

	private static final Logger LOG = LogManager.getLogger(AccessVisitor.class);

	/**
	 * Pairs the updated {@code access} expression produced by a trailer with
	 * the value that should be recorded in the trailer chain slot. These differ
	 * only for the {@code super()} synthesis, where the chain stores
	 * {@code self} (so the next trailer can use it as the receiver) while
	 * {@code access} carries the synthesised {@code super(Class, self)} call.
	 */
	private record TrailerOutcome(
			Expression access,
			Expression chainSlot) {
		static TrailerOutcome of(
				Expression e) {
			return new TrailerOutcome(e, e);
		}
	}

	private final ParserContext ctx;
	private final ParserSupport support;

	public AccessVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Expression visitAtom_expr(
			Atom_exprContext pctx) {
		if (pctx.AWAIT() != null)
			support.unsound(pctx, "await stripped");
		if (pctx.trailer().isEmpty())
			return ctx.expr().visitAtom(pctx.atom());
		return resolveTrailers(pctx);
	}

	private Expression resolveTrailers(
			Atom_exprContext pctx) {
		Expression[] chain = new Expression[pctx.trailer().size() + 1];
		int i = 0;
		Expression access = ctx.expr().visitAtom(pctx.atom());
		chain[i] = access;
		i++;
		for (TrailerContext trailer : pctx.trailer()) {
			TrailerOutcome out = applyTrailer(chain, i, access, trailer);
			access = out.access();
			chain[i] = out.chainSlot();
			i++;
		}
		return access;
	}

	private TrailerOutcome applyTrailer(
			Expression[] chain,
			int i,
			Expression access,
			TrailerContext t) {
		if (t.NAME() != null)
			return TrailerOutcome.of(applyAttribute(access, t));
		if (t.OPEN_PAREN() != null)
			return applyCall(chain, i, access, t);
		if (t.OPEN_BRACK() != null)
			return TrailerOutcome.of(applySubscript(access, t));
		throw new UnsupportedStatementException();
	}

	private Expression applyAttribute(
			Expression base,
			TrailerContext t) {
		return new AttributeAccess(ctx.currentCFG(), support.getLocation(t), base, t.NAME().getText());
	}

	private Expression applySubscript(
			Expression base,
			TrailerContext t) {
		List<Subscript_Context> subs = t.subscriptlist().subscript_();
		if (subs.size() != 1) {
			support.unsound(t, "multiple subscripts not supported");
			return base;
		}
		Expression key = visitSubscript_(subs.get(0));
		Expression getitemAttr = new AttributeAccess(ctx.currentCFG(), support.getLocation(t), base,
				DunderMethods.GETITEM);
		return new FunctionApply(ctx.currentCFG(), support.getLocation(t), getitemAttr,
				new Expression[] { base, key }, true);
	}

	private TrailerOutcome applyCall(
			Expression[] chain,
			int i,
			Expression access,
			TrailerContext t) {
		boolean receiverPrepended = (i >= 2);
		if (isZeroArgSuperCall(access, t, receiverPrepended)) {
			PyClassUnit enclosing = findEnclosingPyClassUnit();
			if (enclosing != null)
				return synthesizeSuperCall(enclosing, t);
			LOG.warn("super() at {} has no resolvable enclosing class — emitting best-effort call",
					support.getLocation(t));
		}
		List<Expression> args = new ArrayList<>();
		if (receiverPrepended)
			args.add(chain[i - 2]);
		if (t.arglist() != null)
			for (ArgumentContext arg : t.arglist().argument())
				args.add(visitArgument(arg));
		args = support.convertAssignmentsToByNameParameters(args);
		Expression call = new FunctionApply(ctx.currentCFG(), support.getLocation(t), access,
				args.toArray(Expression[]::new), receiverPrepended);
		return TrailerOutcome.of(call);
	}

	private boolean isZeroArgSuperCall(
			Expression access,
			TrailerContext t,
			boolean receiverPrepended) {
		return access instanceof PyNameRef superRef
				&& "super".equals(superRef.getName())
				&& t.arglist() == null
				&& !receiverPrepended
				&& ctx.objectUnit() != null;
	}

	private TrailerOutcome synthesizeSuperCall(
			PyClassUnit enclosing,
			TrailerContext t) {
		String fullName = enclosing.getName();
		String simpleName = fullName.contains(".")
				? fullName.substring(fullName.lastIndexOf('.') + 1)
				: fullName;
		Expression classRef = support.makeRef(simpleName, support.getLocation(t));
		LOG.debug("super() synthesis in '{}': simpleName='{}', classRef='{}' id={}",
				ctx.currentUnit().getName(), simpleName, classRef,
				System.identityHashCode(classRef));
		String firstParamName = ctx.currentCFG().getDescriptor().getFormals().length > 0
				? ctx.currentCFG().getDescriptor().getFormals()[0].getName()
				: "self";
		Expression selfRef = new VariableRef(ctx.currentCFG(), support.getLocation(t), firstParamName);
		Expression superFunc = support.makeScopedAttributeRef(ctx.objectUnit(), "super",
				support.getLocation(t));
		Expression call = new FunctionApply(ctx.currentCFG(), support.getLocation(t), superFunc,
				new Expression[] { classRef, selfRef }, false);
		LOG.debug("super() FunctionApply created: id={}", System.identityHashCode(call));
		// original self as chain-slot receiver; super proxy only carries type
		// resolution forward
		return new TrailerOutcome(call, selfRef);
	}

	/**
	 * Returns the {@link PyClassUnit} that lexically encloses the current
	 * function being parsed, or {@code null} if the current function is not
	 * defined inside a Python class.
	 */
	public PyClassUnit findEnclosingPyClassUnit() {
		String funcName = ctx.currentUnit().getName();
		int lastDot = funcName.lastIndexOf('.');
		if (lastDot <= 0)
			return null;
		String classUnitName = funcName.substring(0, lastDot);
		Unit candidate = ctx.program().getUnit(classUnitName);
		LOG.debug("findEnclosingPyClassUnit: funcName='{}', classUnitName='{}', candidate={}",
				funcName, classUnitName, candidate == null ? "null" : candidate.getName());
		if (candidate instanceof PyClassUnit pcu)
			return pcu;
		return null;
	}

	public Expression visitArgument(
			ArgumentContext pctx) {
		if (pctx.ASSIGN() != null) {
			boolean prev = ctx.shouldPrependUnitAccess();
			ctx.shouldPrependUnitAccess(false);
			Expression left = ctx.expr().visitTest(pctx.test(0));
			ctx.shouldPrependUnitAccess(prev);
			Expression right = ctx.expr().visitTest(pctx.test(1));
			return new it.unive.pylisa.cfg.expression.PyAssign(ctx.currentCFG(), support.getLocation(pctx), left,
					right);
		}
		if (pctx.STAR() != null)
			return new StarExpression(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitTest(pctx.test(0)));
		if (pctx.comp_for() != null || pctx.POWER() != null || pctx.test().size() != 1)
			return new Empty(ctx.currentCFG(), support.getLocation(pctx));
		return ctx.expr().visitTest(pctx.test(0));
	}

	public Expression visitSubscript_(
			Subscript_Context pctx) {
		if (pctx.COLON() != null) {
			SourceCodeLocation loc = support.getLocation(pctx);
			Expression left = pctx.test1() == null ? new Empty(ctx.currentCFG(), loc)
					: ctx.expr().visitTest(pctx.test1().test());
			Expression middle = pctx.test2() == null ? new Empty(ctx.currentCFG(), loc)
					: ctx.expr().visitTest(pctx.test2().test());
			Expression right = pctx.sliceop() == null || pctx.sliceop().test() == null
					? new Empty(ctx.currentCFG(), loc)
					: ctx.expr().visitTest(pctx.sliceop().test());
			return new RangeValue(ctx.currentCFG(), loc, left, middle, right);
		}
		return ctx.expr().visitTest(pctx.test());
	}

	public List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext pctx) {
		List<Expression> result = new ArrayList<>();
		if (pctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : pctx.subscript_())
			result.add(visitSubscript_(e));
		return result;
	}
}
