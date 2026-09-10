package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.evaluation.RightToLeftEvaluation;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class PyAssign extends Assignment {

	public PyAssign(
			CFG cfg,
			CodeLocation location,
			Expression target,
			Expression expression) {
		super(cfg, location, RightToLeftEvaluation.INSTANCE, target, expression);
	}

	/**
	 * Overridden (rather than handling {@code x[i] = v} in
	 * {@link #fwdBinarySemantics}) so that the target {@code x[i]} is never
	 * evaluated as a whole: the default {@code NaryExpression.forwardSemantics}
	 * flow evaluates every sub-expression &mdash; including the left-hand
	 * side &mdash; before {@code fwdBinarySemantics} is even invoked, and
	 * {@code x[i]} as a {@link PySingleArrayAccess} would dispatch
	 * {@code __getitem__} (a real read). For an assignment target that read
	 * result is discarded anyway (it is not a valid write target), but
	 * actually performing it is observable: e.g. an out-of-range constant
	 * index on a {@code Tuple} would incorrectly raise {@code IndexError}
	 * (from the discarded read) instead of the {@code TypeError} that
	 * {@code __setitem__} itself raises for any tuple mutation, regardless
	 * of the index. So for this target shape, only {@code x} and {@code i}
	 * (the receiver and the index, i.e. {@code access.getLeft()}/
	 * {@code access.getRight()}) are evaluated individually, never the
	 * {@code PySingleArrayAccess} node itself.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Expression lefthand = getLeft();
		if (!(lefthand instanceof PySingleArrayAccess))
			return super.forwardSemantics(entryState, interprocedural, expressions);

		// x[i] = v  ~>  x.__setitem__(i, v)
		PySingleArrayAccess access = (PySingleArrayAccess) lefthand;
		Expression selfExpr = access.getLeft();
		Expression indexExpr = access.getRight();
		Expression valueExpr = getRight();

		// right-to-left, mirroring this class's declared evaluation order:
		// value first, then self/index (in the same left-to-right order
		// PySingleArrayAccess itself would use for them)
		AnalysisState<A> valueState = evalSub(valueExpr, entryState, interprocedural, expressions);
		AnalysisState<A> selfState = evalSub(selfExpr, valueState, interprocedural, expressions);
		AnalysisState<A> indexState = evalSub(indexExpr, selfState, interprocedural, expressions);

		ExpressionSet valueIds = valueState.getExecutionExpressions();
		ExpressionSet selfIds = selfState.getExecutionExpressions();
		ExpressionSet indexIds = indexState.getExecutionExpressions();

		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		SymbolAliasing aliasing = indexState.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		Set<Type> rtsValue = new HashSet<>();
		for (SymbolicExpression valueSym : valueIds)
			rtsValue.addAll(analysis.getRuntimeTypesOf(indexState, valueSym, this));

		AnalysisState<A> result = indexState.bottom();
		boolean resolved = false;
		for (SymbolicExpression selfSym : selfIds) {
			Set<Type> rtsSelf = analysis.getRuntimeTypesOf(indexState, selfSym, this);
			Set<Type> rtsIndex = indexIds.isEmpty() ? Set.of(Untyped.INSTANCE) : null;
			for (SymbolicExpression indexSym : indexIds)
				rtsIndex = rtsIndex == null
						? analysis.getRuntimeTypesOf(indexState, indexSym, this)
						: rtsIndex;

			for (Type tSelf : rtsSelf) {
				UnresolvedCall setitem = new UnresolvedCall(
						getCFG(),
						loc,
						CallType.STATIC,
						null,
						"__setitem__",
						LeftToRightEvaluation.INSTANCE,
						selfExpr,
						indexExpr,
						valueExpr);
				try {
					interprocedural.resolve(setitem,
							new Set[] { Collections.singleton(tSelf), rtsIndex, rtsValue }, aliasing);
					resolved = true;
					result = result.lub(setitem.forwardSemantics(indexState, interprocedural, expressions));
				} catch (CallResolutionException e) {
					// this type does not support item assignment: it does not contribute
				}
			}
		}

		if (!resolved)
			// no type implements __setitem__: real Python raises TypeError,
			// which is not modeled here
			throw new UnsupportedStatementException(this);

		// __setitem__ has no useful return value in Python (None); leave the
		// assigned value on the stack, matching plain scalar assignment
		AnalysisState<A> finalResult = result.bottom();
		for (SymbolicExpression valueSym : valueIds)
			finalResult = finalResult.lub(interprocedural.getAnalysis().smallStepSemantics(result, valueSym, this));
		return finalResult;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> evalSub(
			Expression node,
			AnalysisState<A> preState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> tmp = node.forwardSemantics(preState, interprocedural, expressions);
		expressions.put(node, tmp);
		return tmp;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();

		Analysis<A, D> analysis = interprocedural.getAnalysis();

		Expression lefthand = getLeft();

		if (!(lefthand instanceof TupleCreation))
			return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);

		// get the variables being assigned
		Expression[] vars = ((TupleCreation) lefthand).getSubExpressions();
		List<ExpressionSet> ids = Arrays.stream(vars)
				.map(v -> expressions.getState(v).getExecutionExpressions()).collect(Collectors.toList());

		// assign the pairs
		AnalysisState<A> assign = state;

		Type type = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);
		HeapReference ref = new HeapReference(type, right, loc);
		HeapDereference deref = new HeapDereference(type, ref, loc);

		for (int i = 0; i < ids.size(); i++) {
			ExpressionSet id = ids.get(i);

			AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref,
					new Constant(Int32Type.INSTANCE, i, loc),
					loc);
			AnalysisState<A> fieldState = analysis.smallStepSemantics(assign, fieldAcc, this);

			AnalysisState<A> fieldResult = state.bottom();
			for (SymbolicExpression single : id)
				for (SymbolicExpression lenId : fieldState.getExecutionExpressions())
					fieldResult = fieldResult.lub(analysis.assign(fieldState, single, lenId, this));
			assign = assign.lub(fieldResult);
		}

		// we leave the reference on the stack
		return interprocedural.getAnalysis().smallStepSemantics(assign, ref, this);
	}
}