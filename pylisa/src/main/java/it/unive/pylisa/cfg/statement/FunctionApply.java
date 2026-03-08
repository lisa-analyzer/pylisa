package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.VariadicExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class FunctionApply extends NaryExpression {
	Expression identifier;
	private final boolean hasReceiver;

	public FunctionApply(
			CFG cfg,
			CodeLocation location,
			Expression identifier,
			Expression[] params) {
		this(cfg, location, identifier, params, false);
	}

	public FunctionApply(
			CFG cfg,
			CodeLocation location,
			Expression identifier,
			Expression[] params,
			boolean hasReceiver) {

		super(cfg, location, "$FunctionApply", prependReceiver(params, identifier));
		this.identifier = identifier;
		this.hasReceiver = hasReceiver;
	}

	@Override
	public String toString() {
		return getSubExpressions()[0] + "("
				+ Arrays.stream(getSubExpressions()).toList().subList(1, getSubExpressions().length) + ")";
	}

	private static Expression[] prependReceiver(
			Expression[] params,
			Expression receiver) {
		Expression[] result = new Expression[params.length + 1];
		result[0] = receiver;
		System.arraycopy(params, 0, result, 1, params.length);
		return result;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		FunctionApply other = (FunctionApply) o;
		int cmp = Boolean.compare(hasReceiver, other.hasReceiver);
		if (cmp != 0)
			return cmp;
		cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		boolean anyTypeFound = false;
		org.apache.logging.log4j.LogManager.getLogger(FunctionApply.class).debug(
				"forwardSemanticsAux: this id={} subexpr={}", System.identityHashCode(this),
				java.util.Arrays.asList(getSubExpressions()).stream().map(e -> e + "/" + System.identityHashCode(e)).toList());
		for (SymbolicExpression identifier : params[0]) {
			Set<Type> runtimeTypes = interprocedural.getAnalysis().getRuntimeTypesOf(state, identifier, this);
			boolean handledIdentifier = false;
			if (identifier instanceof LazyEvaluatedVariadicExpression lazyEval) {
				anyTypeFound = true;
				SymbolicExpression e = lazyEval.getExpression();
				if (e instanceof VariadicExpression ve) {
					VariadicExpression.Builder builder = new VariadicExpression.Builder()
							.operator(ve.getOperator())
							.staticType(ve.getStaticType())
							.location(ve.getCodeLocation());
					SymbolicExpression[] oldOperands = ve.getOperands();
					ve.getVarargsIndex().forEach((
							name,
							index) -> {
						builder.varargsOperand(name, oldOperands[index]);
					});
					int j = 1;
					for (Expression le : lazyEval.additionalExpressions) {
						builder.varargsOperand(le.toString(), params[j].elements.stream().findFirst().get()); // todo:
																												// fix
																												// that!
					}
					VariadicExpression expr = builder.build();
					return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
				}
			} else {
				for (Type t : runtimeTypes) {
					if (t instanceof PyClassType pct) {
						handledIdentifier = true;
						anyTypeFound = true;
						Expression[] classParams;
						if (hasReceiver && getSubExpressions().length > 1) {
							// getSubExpressions() = [identifier, receiver,
							// arg1, arg2, ...]
							// ClassInstantiation should receive: [identifier,
							// arg1, arg2, ...]
							int len = getSubExpressions().length;
							classParams = new Expression[len - 1];
							classParams[0] = getSubExpressions()[0];
							System.arraycopy(getSubExpressions(), 2, classParams, 1, len - 2);
						} else {
							classParams = getSubExpressions();
						}
						ClassInstantiation ci = new ClassInstantiation(this.getCFG(), getLocation(), pct,
								classParams);
						result = result.lub(ci.forwardSemantics(state, interprocedural, expressions));
					}
					if (t instanceof PyFunctionType pft) {
						handledIdentifier = true;
						anyTypeFound = true;
						CodeMember cm = pft.getUnit().getFunction();
						// here, maybe I need to add the main
						Call c = null;
						if (cm instanceof NativeCFG cfg) {
							c = new NativeCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call",
									List.of(cfg),
									Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
							org.apache.logging.log4j.LogManager.getLogger(FunctionApply.class).debug(
									"NativeCall built: this(FA)id={} subexpr[1]={} id={}, NativeCFG={}",
									System.identityHashCode(this),
									getSubExpressions().length > 1 ? getSubExpressions()[1] : "n/a",
									getSubExpressions().length > 1 ? System.identityHashCode(getSubExpressions()[1]) : -1,
									cfg.getDescriptor().getName());
						} else if (cm instanceof CFG cfg) {
							c = new CFGCall(this.getCFG(), getLocation(), Call.CallType.STATIC, "", "$call",
									List.of(cfg),
									Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
						}
						if (c != null) {
							AnalysisState<A> callResult = c.forwardSemantics(state, interprocedural, expressions);
							if (callResult.isBottom()) {
								result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state,
										new PushAny(Untyped.INSTANCE, getLocation()), this));
							} else {
								result = result.lub(callResult);
							}
						} else {
							// If we know this is a function value but cannot
							// resolve a
							// concrete callable body, stay conservative without
							// collapsing to bottom.
							result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state,
									new PushAny(Untyped.INSTANCE, getLocation()), this));
						}
					}
					// AttributeAccess access = new
					// AttributeAccess(this.getCFG(),
					// SyntheticLocation.INSTANCE, getSubExpressions()[0],
					// "__new__");

					// FunctionApply apply = new FunctionApply(getCFG(),
					// SyntheticLocation.INSTANCE, access, new Expression[]{});
					// result = result.lub(apply.forwardSemantics(state,
					// interprocedural, expressions));
				}
				if (!handledIdentifier) {
					result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state,
							new PushAny(Untyped.INSTANCE, getLocation()), this));
					anyTypeFound = true;
				}
				// result = result.lub(analysis.smallStepSemantics(state,
				// identifier, this));
			}
		}
		if (!anyTypeFound) {
			return interprocedural.getAnalysis().smallStepSemantics(state,
					new PushAny(Untyped.INSTANCE, getLocation()), this);
		}
		// result.getExecution().getComputedExpressions().forEach())

		/*
		 * if (right instanceof Constant) { Constant constant = (Constant)
		 * right; if (left.getStaticType() instanceof FunctionLiteral l) { CFG
		 * leftCFG = l.getValue(); CFGCall call = new CFGCall(this.getCFG(),
		 * getLocation(), Call.CallType.UNKNOWN, (String) null,
		 * leftCFG.getDescriptor().getName(), List.of(leftCFG),
		 * this.getRight()); } if (left instanceof
		 * LazyEvaluatedVariadicExpression lazyEval) { SymbolicExpression e =
		 * lazyEval.getExpression(); if (e instanceof VariadicExpression ve) {
		 * VariadicExpression.Builder builder = new VariadicExpression.Builder()
		 * .operator(ve.getOperator()) .staticType(ve.getStaticType())
		 * .location(ve.getCodeLocation()); SymbolicExpression[] oldOperands =
		 * ve.getOperands(); /*for (SymbolicExpression op : oldOperands) {
		 * builder.operand(op); } ve.getVarargsIndex().forEach((name, index) ->
		 * { builder.varargsOperand(name, oldOperands[index]); }); for
		 * (Expression le : lazyEval.additionalExpressions) {
		 * builder.varargsOperand(le.toString(), right); } VariadicExpression
		 * expr = builder.build(); return
		 * interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
		 * } } }
		 */

		return result;
	}
}
