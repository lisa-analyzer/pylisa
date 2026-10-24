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
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.RightToLeftEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

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
	 * flow evaluates every sub-expression &mdash; including the left-hand side
	 * &mdash; before {@code fwdBinarySemantics} is even invoked, and
	 * {@code x[i]} as a {@link PySingleArrayAccess} would dispatch
	 * {@code __getitem__} (a real read). For an assignment target that read
	 * result is discarded anyway (it is not a valid write target), but actually
	 * performing it is observable: e.g. an out-of-range constant index on a
	 * {@code Tuple} would incorrectly raise {@code IndexError} (from the
	 * discarded read) instead of the {@code TypeError} that {@code __setitem__}
	 * itself raises for any tuple mutation, regardless of the index. So for
	 * this target shape, only {@code x} and {@code i} (the receiver and the
	 * index, i.e. {@code access.getLeft()}/ {@code access.getRight()}) are
	 * evaluated individually, never the {@code PySingleArrayAccess} node
	 * itself.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Expression lefthand = getLeft();
		if (lefthand instanceof TupleCreation)
			// a, b = ... (the targets are not evaluated)
			return unpackAssignment(entryState, interprocedural, expressions);
		if (!(lefthand instanceof PySingleArrayAccess))
			return super.forwardSemantics(entryState, interprocedural, expressions);

		// x[i] = v ~> x.__setitem__(i, v)
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

		if (indexState.getExecutionState().isBottom())
			// unreachable code (e.g. a branch excluded by its condition), or
			// the evaluation of the value, the receiver, or the index always
			// raises an error: nothing is assigned
			return indexState;

		ExpressionSet valueIds = valueState.getExecutionExpressions();
		ExpressionSet selfIds = selfState.getExecutionExpressions();
		ExpressionSet indexIds = indexState.getExecutionExpressions();

		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		SymbolAliasing aliasing = indexState.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		Set<Type> rtsValue = new HashSet<>();
		for (SymbolicExpression valueSym : valueIds)
			rtsValue.addAll(analysis.getRuntimeTypesOf(indexState, valueSym, this));

		AnalysisState<A> result = indexState.bottomExecution();
		boolean resolved = false;
		List<Call> setitems = new ArrayList<>();
		// the arguments have already been evaluated: the call must not
		// evaluate them again (e.g., calling f twice in x[i] = f())
		ExpressionSet[] args = { selfIds, indexIds, valueIds };
		for (SymbolicExpression selfSym : selfIds) {
			Set<Type> rtsSelf = analysis.getRuntimeTypesOf(indexState, selfSym, this);
			Set<Type> rtsIndex = indexIds.isEmpty() ? Set.of(Untyped.INSTANCE) : null;
			for (SymbolicExpression indexSym : indexIds)
				rtsIndex = rtsIndex == null
						? analysis.getRuntimeTypesOf(indexState, indexSym, this)
						: rtsIndex;

			for (Type tSelf : rtsSelf) {
				// the methods of the classes of the program are instance
				// methods, looked up in the method resolution order of the
				// class (see PyMro), while the natives of the library classes
				// (e.g. list) take the receiver as their first argument
				CallType kind = tSelf.isPointerType() && tSelf.asPointerType().getInnerType().isUnitType()
						&& ClassBases.isProgramClass(tSelf.asPointerType().getInnerType().asUnitType().getUnit())
								? CallType.INSTANCE
								: CallType.STATIC;
				UnresolvedCall setitem = CallSites.get(interprocedural, this, kind, null, "__setitem__",
						selfExpr, indexExpr, valueExpr);
				try {
					Call call = interprocedural.resolve(setitem,
							new Set[] { Collections.singleton(tSelf), rtsIndex, rtsValue }, aliasing);
					resolved = true;
					if (call instanceof OpenCall && PyBinaryDispatch.fullyKnown(tSelf)) {
						// all the methods of the class of the receiver are
						// known, and none is a __setitem__ accepting the
						// arguments (e.g. an int, a str, or None)
						result = result.lub(PyExceptions.raise(analysis, indexState, getCFG(), loc, this,
								LibrarySpecificationProvider.TYPE_ERROR));
						continue;
					}
					result = result.lub(call.forwardSemanticsAux(interprocedural, indexState, args, expressions));
					setitems.add(call);
				} catch (CallResolutionException e) {
					// this type does not support item assignment: it does not
					// contribute
				}
			}
		}

		if (!resolved)
			// the call cannot be resolved for any type of the receiver
			throw new UnsupportedStatementException(this);

		// as sub-expressions are evaluated here, their meta variables (e.g.,
		// the return value of a call) are propagated to this assignment, as
		// done by NaryExpression
		Collection<Identifier> metas = getMetaVariables();
		for (Expression sub : List.of(valueExpr, selfExpr, indexExpr)) {
			metas.addAll(sub.getMetaVariables());
			sub.getMetaVariables().clear();
		}
		for (Call setitem : setitems)
			metas.addAll(setitem.getMetaVariables());

		if (getRootStatement() == this)
			// the value of the statement is not used: the meta variables are
			// not needed anymore, and nothing is left on the stack
			return analysis.smallStepSemantics(result, new Skip(loc), this).forgetIdentifiers(metas, this);

		// __setitem__ has no useful return value in Python (None); leave the
		// assigned value on the stack, matching plain scalar assignment, as it
		// is used by the enclosing assignment (e.g., a = x[i] = f()), that
		// will forget the meta variables
		AnalysisState<A> finalResult = result.bottom();
		for (SymbolicExpression valueSym : valueIds)
			finalResult = finalResult.lub(analysis.smallStepSemantics(result, valueSym, this));
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
		AnalysisState<A> result = assign(interprocedural, state, left, right, expressions);
		if (getLeft() instanceof TupleCreation && getRootStatement() != this)
			// the reference to the right-hand side is left on the stack, and
			// it is used by the enclosing assignment (e.g., x = a, b = f()),
			// that will forget the meta variables
			return result;
		// the value of the right-hand side is consumed by the assignment: its
		// meta variables (e.g., the return value of a call) are not needed
		// anymore
		return result.forgetIdentifiers(getRight().getMetaVariables(), this);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> assign(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result;
		if (left instanceof Identifier && left.equals(right))
			// x = x (e.g. x |= y, where __ior__ returns x itself) does not
			// change anything, while assigning would lose what is known about
			// the object x points to, as the heap drops it when x is unbound
			result = interprocedural.getAnalysis().smallStepSemantics(state, left, this);
		else
			// tuple targets are unpacked by unpackAssignment
			result = super.fwdBinarySemantics(interprocedural, state, left, right, expressions);
		return bind(getLeft(), result);
	}

	// binds the name of the target, if it is a name (see PyNameRef)
	private static <A extends AbstractLattice<A>> AnalysisState<A> bind(
			Expression target,
			AnalysisState<A> state) {
		return target instanceof PyNameRef ? ((PyNameRef) target).bind(state) : state;
	}

	// a, b = value: the value is evaluated, and unpacked into the targets
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unpackAssignment(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		Expression valueExpr = getRight();
		AnalysisState<A> valueState = evalSub(valueExpr, entryState, interprocedural, expressions);

		AnalysisState<A> result = valueState.bottomExecution();
		for (SymbolicExpression value : valueState.getExecutionExpressions())
			result = result.lub(unpack(interprocedural, valueState, getLeft(), value, expressions));
		expressions.put(getLeft(), result);

		Collection<Identifier> metas = getMetaVariables();
		metas.addAll(valueExpr.getMetaVariables());
		valueExpr.getMetaVariables().clear();
		if (getRootStatement() == this)
			// nothing is left on the stack
			return analysis.smallStepSemantics(result, new Skip(loc), this).forgetIdentifiers(metas, this);
		// the value is used by the enclosing assignment (e.g., x = a, b = f()),
		// that will forget the meta variables
		AnalysisState<A> finalResult = result.bottomExecution();
		for (SymbolicExpression value : valueState.getExecutionExpressions())
			finalResult = finalResult.lub(analysis.smallStepSemantics(result, value, this));
		return finalResult;
	}

	// assigns value to target, unpacking it if target is a tuple of targets
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unpack(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			Expression target,
			SymbolicExpression value,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		if (target instanceof VariableRef && !(target instanceof PyNameRef && ((PyNameRef) target).cell() != null))
			return bind(target, analysis.assign(state, ((VariableRef) target).getIdentifier(), value, this));
		if (target instanceof PyAttributeAccess || target instanceof PyNameRef) {
			// o.x, or a name held in a cell, is evaluated as the target of the
			// assignment
			AnalysisState<A> evaluated = target.forwardSemantics(state, interprocedural, expressions);
			expressions.put(target, evaluated);
			AnalysisState<A> result = evaluated.bottomExecution();
			for (SymbolicExpression id : evaluated.getExecutionExpressions())
				result = result.lub(analysis.assign(evaluated, id, value, this));
			return bind(target, result.forgetIdentifiers(target.getMetaVariables(), this));
		}
		if (!(target instanceof TupleCreation))
			throw new UnsupportedStatementException("Unpacking into " + target + " is not supported");

		Expression[] targets = ((TupleCreation) target).getSubExpressions();
		int star = -1;
		for (int i = 0; i < targets.length; i++)
			if (targets[i] instanceof StarExpression)
				if (star >= 0)
					throw new UnsupportedStatementException("Multiple starred targets: " + target);
				else
					star = i;
		return Unpacking.unpack(interprocedural, state, value, targets, star, this,
				(
						s,
						t,
						v) -> unpack(interprocedural, s, t, v, expressions));
	}
}
