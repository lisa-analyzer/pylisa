package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.InstrumentedReceiverRef;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.debug.ConstructorResolutionTrace;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The instantiation of a class reached by a {@link PyCall}, performed as
 * Python performs it, by two calls that the call graph resolves and records
 * like any other: {@code __new__} creates the object, then {@code __init__}
 * initialises it. {@code __init__} is looked up in the state each target of
 * {@code __new__} leaves, so a {@code __new__} that rebinds it is taken into
 * account path by path. The result is the created object, not what
 * {@code __init__} returns.
 * <p>
 * When {@code __init__} cannot be run (the created value is not a named
 * object, or {@code __new__} creates no value), the call to it is still
 * resolved, so that what cannot be run is reported, and the object is
 * unknown.
 * </p>
 */
public class PyInstantiation extends NaryExpression {

	private final PyClassType classType;

	/**
	 * The calls this instantiation performs, one per method and class that
	 * defines it, built once so that each is the same call site in every
	 * fixpoint iteration.
	 */
	private final Map<Method, PyCall> calls = new HashMap<>();

	/**
	 * A method of the class, as defined by a class.
	 */
	private record Method(String name, CompilationUnit owner) {
	}

	/**
	 * Builds an instantiation.
	 *
	 * @param cfg       the CFG the instantiation belongs to
	 * @param location  the location of the call that instantiates
	 * @param classType the class
	 * @param operands  the class expression, then the constructor arguments
	 */
	public PyInstantiation(
			CFG cfg,
			CodeLocation location,
			PyClassType classType,
			Expression[] operands) {
		super(cfg, location, "$PyInstantiation", classType, operands);
		this.classType = classType;
	}

	/**
	 * Yields the class this instantiation creates an instance of.
	 *
	 * @return the class
	 */
	public PyClassType getClassType() {
		return classType;
	}

	/**
	 * Yields the call of a method of the class, as defined by its owner (the
	 * class or the ancestor that defines it), with the operands of this
	 * instantiation: {@code __new__} is passed the class and the constructor
	 * arguments; {@code __init__} is passed the object being created as its
	 * receiver, so that readers of the call find the object there, and the
	 * constructor arguments.
	 */
	private PyCall call(
			String method,
			CompilationUnit owner) {
		return calls.computeIfAbsent(new Method(method, owner), key -> {
			PythonScopedAttributeAccessRef callee = new PythonScopedAttributeAccessRef(getCFG(), getLocation(), owner,
					new Global(getLocation(), owner, method, false));
			Expression[] operands = getSubExpressions();
			PyCall call;
			if (method.equals("__init__")) {
				Expression[] arguments = Arrays.copyOf(operands, operands.length);
				arguments[0] = new InstrumentedReceiverRef(getCFG(), getLocation(), false);
				call = new PyCall(getCFG(), getLocation(), callee, arguments, true);
			} else
				call = new PyCall(getCFG(), getLocation(), callee, operands.clone());
			call.setParentStatement(this);
			return call;
		});
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CallTargets.Resolution creation = CallTargets.attribute(interprocedural.getAnalysis(), state, classType,
				"__new__", this);
		recordResolution("__new__", creation);
		PyCall create = call("__new__", creation.owner() == null ? classType.getUnit() : creation.owner());
		// the operands of __new__ are those of this instantiation, already
		// evaluated: only its callee is evaluated here
		Expression callee = create.getSubExpressions()[0];
		AnalysisState<A> afterCallee = callee.forwardSemantics(state, interprocedural, expressions);
		expressions.put(callee, afterCallee);
		ExpressionSet[] createParams = new ExpressionSet[params.length + 1];
		createParams[0] = afterCallee.getExecutionExpressions();
		System.arraycopy(params, 0, createParams, 1, params.length);
		PyResolvedCall resolved = create.resolve(interprocedural, state, expressions);
		AnalysisState<A> result = state.bottomExecution();
		ExpressionSet objects = new ExpressionSet().bottom();
		// each target of __new__ creates the object on its own: __init__ is
		// looked up in the state each one leaves, never in their join, where
		// a binding made on one path would hide the inherited __init__ of
		// another
		for (AnalysisState<A> created : resolved.applyEach(interprocedural, state, createParams, expressions,
				false)) {
			boolean reachable = !created.getExecution().isBottom() && !created.getExecutionState().isBottom();
			if (reachable && created.getExecutionExpressions().isEmpty()) {
				// __new__ returns normally without a value: the object is
				// unknown, and __init__ is not run
				AnalysisState<A> unknown = unknownObject(interprocedural, created);
				result = result.lub(unknown);
				objects = objects.lub(unknown.getExecutionExpressions());
				notRun(interprocedural, created);
			}
			// the object exists once __init__ returns: an __init__ that always
			// raises leaves no execution after it, and what __init__ returns is
			// not the object
			AnalysisState<A> initialized = created.bottomExecution();
			for (SymbolicExpression value : created.getExecutionExpressions())
				if (value instanceof Identifier) {
					objects = objects.lub(new ExpressionSet(value));
					initialized = initialized.lub(initialize(interprocedural, created, expressions));
				} else {
					// __init__ cannot be run on a value that is not a named
					// location: the object is unknown
					AnalysisState<A> unknown = unknownObject(interprocedural, created);
					initialized = initialized.lub(unknown);
					objects = objects.lub(unknown.getExecutionExpressions());
					notRun(interprocedural, created);
				}
			result = result.lub(initialized);
		}
		return result.withExecutionExpressions(objects);
	}

	/**
	 * Runs {@code __init__} on the object just created, with the arguments of
	 * this instantiation, looked up in the state {@code __new__} left.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> initialize(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> created,
			StatementStore<A> expressions)
			throws SemanticException {
		return initCall(interprocedural, created).forwardSemantics(created, interprocedural, expressions);
	}

	/**
	 * Resolves {@code __init__} without running it, so that the call graph
	 * reports it as a part of the call that cannot be run. Running it would
	 * evaluate the constructor arguments once more.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> void notRun(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> created)
			throws SemanticException {
		PyCall init = initCall(interprocedural, created);
		@SuppressWarnings("unchecked")
		Set<Type>[] types = new Set[init.getSubExpressions().length];
		Arrays.fill(types, Set.of(NoInfoType.INSTANCE));
		try {
			interprocedural.resolve(init, types, created.getExecutionInfo(SymbolAliasing.INFO_KEY,
					SymbolAliasing.class));
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve " + init + " at " + getLocation(), e);
		}
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> PyCall initCall(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> created)
			throws SemanticException {
		CallTargets.Resolution initialization = CallTargets.attribute(interprocedural.getAnalysis(), created,
				classType, "__init__", this);
		return call("__init__", initialization.owner() == null ? classType.getUnit() : initialization.owner());
	}

	/**
	 * Yields the state where the created object is unknown.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknownObject(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state)
			throws SemanticException {
		return interprocedural.getAnalysis().smallStepSemantics(state, new PushAny(Untyped.INSTANCE, getLocation()),
				this);
	}

	private void recordResolution(
			String attribute,
			CallTargets.Resolution resolution) {
		List<String> ancestors = new ArrayList<>();
		for (CompilationUnit ancestor : classType.getUnit().getImmediateAncestors())
			ancestors.add(ancestor.getName());
		ConstructorResolutionTrace.record(
				getLocation().toString(),
				classType.getUnit().getName(),
				ancestors,
				attribute,
				resolution.lookupPath(),
				resolution.types().toString(),
				resolution.owner() == null ? "<none>" : resolution.owner().getName(),
				resolution.mode());
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		PyInstantiation other = (PyInstantiation) o;
		int cmp = classType.toString().compareTo(other.classType.toString());
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
		// as for calls: an identity tiebreaker would prevent convergence, since
		// instantiations are built anew while analysing, at the same location
		// and with identical operands
		return 0;
	}
}
