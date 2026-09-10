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

		if (lefthand instanceof PySingleArrayAccess) {
			// x[i] = v  ~>  x.__setitem__(i, v). left (the evaluated PySingleArrayAccess,
			// i.e. a __getitem__ read) is discarded: it is not a valid write target,
			// so self/index are re-derived from the access node's own sub-expressions,
			// already evaluated as part of evaluating the (discarded) read.
			PySingleArrayAccess access = (PySingleArrayAccess) lefthand;
			ExpressionSet selfIds = expressions.getState(access.getLeft()).getExecutionExpressions();
			ExpressionSet indexIds = expressions.getState(access.getRight()).getExecutionExpressions();
			SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

			AnalysisState<A> result = state.bottom();
			boolean resolved = false;
			for (SymbolicExpression selfSym : selfIds) {
				Set<Type> rtsSelf = analysis.getRuntimeTypesOf(state, selfSym, this);
				Set<Type> rtsIndex = indexIds.isEmpty() ? Set.of(Untyped.INSTANCE) : null;
				for (SymbolicExpression indexSym : indexIds)
					rtsIndex = rtsIndex == null
							? analysis.getRuntimeTypesOf(state, indexSym, this)
							: rtsIndex;
				Set<Type> rtsValue = analysis.getRuntimeTypesOf(state, right, this);

				for (Type tSelf : rtsSelf) {
					UnresolvedCall setitem = new UnresolvedCall(
							getCFG(),
							loc,
							CallType.STATIC,
							null,
							"__setitem__",
							LeftToRightEvaluation.INSTANCE,
							access.getLeft(),
							access.getRight(),
							getRight());
					try {
						interprocedural.resolve(setitem,
								new Set[] { Collections.singleton(tSelf), rtsIndex, rtsValue }, aliasing);
						resolved = true;
						result = result.lub(setitem.forwardSemantics(state, interprocedural, expressions));
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
			return interprocedural.getAnalysis().smallStepSemantics(result, right, this);
		}

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