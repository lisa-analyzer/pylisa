package it.unive.pylisa.libraries.collections;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.dicts.DictModel;

/**
 * Native implementation of
 * {@code collections.defaultdict(default_factory=None, iterable, **kwargs)}: a
 * new, empty dictionary of class {@code defaultdict} (allocated at a location
 * different from the one of the call, see {@link DerivedLocation}), whose
 * missing keys have the value computed by {@code default_factory} (see
 * {@link DictModel#MISSING}). The value is known when {@code default_factory}
 * is {@code int}, {@code str} or {@code bool}; without a
 * {@code default_factory} (that is, {@code None}), reading a missing key raises
 * {@code KeyError} as for a dictionary. With any other {@code default_factory},
 * or with initial entries, the dictionary is unknown, and any exception might
 * be raised (see {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}).
 */
public class DefaultDictBuiltin extends PyNative {

	private final String type;

	/**
	 * Builds the native.
	 *
	 * @param cfg      the cfg where the native is called
	 * @param location the location of the call
	 * @param name     the name of the native
	 * @param type     the name of the class of the created dictionary
	 * @param params   the arguments of the call
	 */
	protected DefaultDictBuiltin(
			CFG cfg,
			CodeLocation location,
			String name,
			String type,
			Expression[] params) {
		super(cfg, location, name, params);
		this.type = type;
	}

	public static DefaultDictBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DefaultDictBuiltin(cfg, location, "defaultdict", LibrarySpecificationProvider.DEFAULT_DICT,
				exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			// dictionaries are symbolic values (see DictionaryCreation)
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
		// the new dictionary: the state of the arguments is the one where it
		// has been created, whose computed expressions are the dictionary (see
		// semantics)
		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(getLocation(), "dict"),
				type);
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		return super.forwardSemanticsAux(interprocedural, created, params, expressions);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression factory = args[0];
		boolean none = factory.getStaticType().isNullType();
		SymbolicExpression missing = none ? null : defaultOf(factory);
		return create(analysis, context, state, missing, none || missing != null, args[1], args[2]);
	}

	/**
	 * Yields the new dictionary, whose missing keys have the given value.
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the state where the dictionary has been created, whose
	 *                     computed expressions are the dictionary
	 * @param missing  the value of the missing keys, or {@code null} if they
	 *                     raise {@code KeyError}
	 * @param known    whether the dictionary is known (otherwise, it is
	 *                     unknown)
	 * @param iterable the initial entries
	 * @param kwargs   the initial entries passed as keyword arguments
	 *
	 * @return the state where the dictionary is computed
	 *
	 * @throws SemanticException if the analysis fails
	 */
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression missing,
			boolean known,
			SymbolicExpression iterable,
			SymbolicExpression kwargs)
			throws SemanticException {
		// the keyword arguments are a dictionary (see PyAssigningStrategy)
		boolean noKwargs = missing(kwargs)
				|| Integer.valueOf(0).equals(DictModel.length(analysis, context, state, kwargs, st));
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS) || !known
				|| !missing(iterable) || !noKwargs) {
			AnalysisState<A> result = unknown(analysis, context, state, null);
			return result.lub(raise(analysis, context, state, LibrarySpecificationProvider.UNKNOWN_EXCEPTION));
		}
		CodeLocation loc = getLocation();
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression dict : state.getExecutionExpressions()) {
			AnalysisState<A> created = state;
			if (missing != null)
				created = analysis.assign(state,
						new AccessChild(Untyped.INSTANCE, new HeapDereference(PyClassType.lookup(type), dict, loc),
								new GlobalVariable(Untyped.INSTANCE, DictModel.MISSING, loc), loc),
						missing, context, st);
			result = result.lub(compute(analysis, context, created, dict));
		}
		return result;
	}

	// the value computed by the given default factory without arguments, if
	// it is known: the one of the builtin classes int, str and bool
	private SymbolicExpression defaultOf(
			SymbolicExpression factory) {
		if (!(factory instanceof Constant) || !(((Constant) factory).getValue() instanceof NativeCFG))
			return null;
		Object function = ((Constant) factory).getValue();
		CodeLocation loc = getLocation();
		if (function == PyNameRef.builtin(getCFG().getProgram(), "int"))
			return new Constant(Int32Type.INSTANCE, 0, loc);
		if (function == PyNameRef.builtin(getCFG().getProgram(), "str"))
			return new Constant(StringType.INSTANCE, "", loc);
		if (function == PyNameRef.builtin(getCFG().getProgram(), "bool"))
			return new Constant(BoolType.INSTANCE, false, loc);
		return null;
	}
}
