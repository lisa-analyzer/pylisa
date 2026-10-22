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
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.symbolic.PyNoneConstant;
import org.apache.commons.lang3.ArrayUtils;

public class PyNewObj extends NaryExpression {

	/**
	 * Builds the object allocation and initialization.
	 *
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param type       the type of the object that is being created
	 * @param parameters the parameters of the constructor call
	 */
	public PyNewObj(
			CFG cfg,
			SourceCodeLocation location,
			String constructName,
			Type type,
			Expression... parameters) {
		super(cfg, location, constructName, type, parameters);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type type = getStaticType();
		// an abstract class cannot be instantiated
		Satisfiability isAbstract = type.isUnitType() && type.asUnitType().getUnit() instanceof ClassUnit
				? ClassBases.isAbstract(getProgram(), (ClassUnit) type.asUnitType().getUnit())
				: Satisfiability.NOT_SATISFIED;
		if (isAbstract == Satisfiability.SATISFIED)
			return PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR);
		AnalysisState<A> created = create(interprocedural, state, params, expressions);
		if (isAbstract == Satisfiability.UNKNOWN)
			created = created.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		return created;
	}

	// creates the object, and initializes it with __init__
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type type = getStaticType();
		ReferenceType reftype = new ReferenceType(type);
		MemoryAllocation created = new MemoryAllocation(type, getLocation(), false);
		HeapReference ref = new HeapReference(reftype, created, getLocation());

		// we need to add the receiver to the parameters
		VariableRef paramThis = new VariableRef(getCFG(), getLocation(), "$self@" + getLocation(), reftype);
		Expression[] fullExpressions = ArrayUtils.insert(0, getSubExpressions(), paramThis);

		// we also have to add the receiver inside the state
		AnalysisState<A> callstate = paramThis.forwardSemantics(state, interprocedural, expressions);
		AnalysisState<A> tmp = state.bottomExecution();
		for (SymbolicExpression v : callstate.getExecutionExpressions()) {
			AnalysisState<A> assigned = analysis.assign(callstate, v, ref, paramThis);
			if (assigned.getExecutionState().isBottom()) {
				tmp = tmp.lub(assigned);
				continue;
			}
			// the heap tracks the fields accessed on each object (see
			// PyAttributeAccess), but until the first access to any field
			// in the program, the fields are unknown for all objects:
			// accessing __class__, that all objects have, makes the fields
			// of the new object known
			AccessChild cls = new AccessChild(Untyped.INSTANCE, new HeapDereference(type, v, getLocation()),
					new Variable(Untyped.INSTANCE, "__class__", getLocation()), getLocation());
			tmp = tmp.lub(analysis.smallStepSemantics(assigned, cls, paramThis)
					.withExecutionExpressions(assigned.getExecutionExpressions()));
		}
		ExpressionSet[] fullParams = ArrayUtils.insert(0, params, callstate.getExecutionExpressions());
		expressions.put(paramThis, tmp);

		// __init__ is looked up through the receiver, also in the bases of
		// the class (see PyMro)
		UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.INSTANCE, null,
				getConstructName(), fullExpressions);
		Call resolved;
		try {
			resolved = interprocedural.resolve(call, call.parameterTypes(expressions, analysis),
					tmp.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve call " + call, e);
		}

		if (resolved instanceof OpenCall && PyBinaryDispatch.fullyKnown(type)) {
			// all the methods of the class are known (see
			// PyBinaryDispatch.fullyKnown), and none can be called
			CompilationUnit unit = type.asUnitType().getUnit();
			boolean init = !unit.getInstanceCodeMembersByName(getConstructName(), true).isEmpty();
			boolean newObj = !unit.getInstanceCodeMembersByName("__new__", true).isEmpty();
			if (init || (getSubExpressions().length > 0 && !newObj)) {
				// either the arguments do not match the ones of __init__, or
				// there is no __init__ and object.__init__ takes no
				// arguments (unless they are consumed by __new__)
				expressions.forget(paramThis);
				return PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.TYPE_ERROR);
			}
			// no class defines __init__: object.__init__ is called, that does
			// nothing and returns None
			resolved = null;
		}

		AnalysisState<A> sem;
		if (resolved == null)
			sem = analysis.smallStepSemantics(tmp, new PyNoneConstant(getLocation()), this);
		else {
			sem = resolved.forwardSemanticsAux(interprocedural, tmp, fullParams, expressions);
			call.getMetaVariables().addAll(resolved.getMetaVariables());
		}

		// __init__ must return None
		Satisfiability none = returnsNone(analysis, sem, resolved, this);

		if (!call.getMetaVariables().isEmpty())
			sem = sem.forgetIdentifiers(call.getMetaVariables(), this);

		AnalysisState<A> badReturn = none == Satisfiability.SATISFIED ? sem.bottom()
				: PyExceptions.raise(analysis, sem, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.TYPE_ERROR);

		// the result is the instrumented receiver, pointing to the object
		// allocated above: the allocation is not evaluated again, as it would
		// replace the object initialized by __init__ with a fresh one. The
		// receiver is the only reference to the object until the result is
		// used (e.g., assigned), so it is not forgotten here (that would also
		// discard the object): it is forgotten as a meta variable by the
		// statement using the result
		expressions.forget(paramThis);
		AnalysisState<A> result = state.bottomExecution();
		if (none != Satisfiability.NOT_SATISFIED)
			for (SymbolicExpression v : callstate.getExecutionExpressions())
				if (v instanceof Identifier) {
					getMetaVariables().add((Identifier) v);
					result = result.lub(analysis.smallStepSemantics(sem, v, call));
				}

		return result.lub(badReturn);
	}

	// whether the value returned by the resolved __init__ (null for
	// object.__init__) is None; the result of an open call is not known, but it
	// is not checked
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability returnsNone(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Call resolved,
			ProgramPoint pp)
			throws SemanticException {
		if (resolved instanceof OpenCall || state.getExecution().isBottom())
			return Satisfiability.SATISFIED;
		boolean none = false, other = false;
		for (SymbolicExpression e : state.getExecutionExpressions())
			if (e instanceof Skip)
				// no return statement
				none = true;
			else
				for (Type t : analysis.getRuntimeTypesOf(state, e, pp))
					if (t.isNullType())
						none = true;
					else
						other = true;
		if (!other)
			return Satisfiability.SATISFIED;
		return none ? Satisfiability.UNKNOWN : Satisfiability.NOT_SATISFIED;
	}
}