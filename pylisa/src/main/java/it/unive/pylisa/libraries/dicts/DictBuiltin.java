package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * Native implementation of {@code dict(iterable, **kwargs)}: a new dictionary
 * (allocated at a location different from the one of the call, see
 * {@link DerivedLocation}) with the entries of {@code iterable}, and then the
 * keyword arguments. A dictionary {@code iterable} is copied (see
 * {@link DictModel#unpackInto}). Otherwise, {@code iterable} must be an
 * iterable of key-value pairs (see {@link Iteration#elements}), whose number is
 * not known: each of them might be added. A pair that is a tuple or a list of
 * known length 2 adds its first element as key and its second as value, while
 * one of another known length raises {@code ValueError}; for any other element
 * (e.g. a {@code str} of two characters), any entry might be added, and
 * {@code TypeError} or {@code ValueError} might be raised. {@code TypeError} is
 * raised if {@code iterable} might not be iterable (see
 * {@link Iteration#iterable}). The keyword arguments are the entries of a
 * dictionary (see {@code PyAssigningStrategy}). The name {@code dict} is also
 * the type of dictionaries, e.g. in {@code _sv_verifier.check_type(x, dict)}.
 */
public class DictBuiltin extends PyNative {

	protected DictBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "dict", params);
	}

	public static DictBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictBuiltin(cfg, location, exprs);
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
			return interprocedural.getAnalysis().smallStepSemantics(state,
					new PushAny(Untyped.INSTANCE, getLocation()), st);
		// the new dictionary: the state of the arguments is the one where it
		// has been created, whose computed expressions are the dictionary (see
		// semantics)
		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(getLocation(), "dict"));
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		return super.forwardSemanticsAux(interprocedural, created, params, expressions);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression iterable = args[0], kwargs = args[1];
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression dict : state.getExecutionExpressions()) {
			AnalysisState<A> filled = state;
			if (!missing(iterable)) {
				Set<Type> types = analysis.getRuntimeTypesOf(state, iterable, st);
				if (!types.isEmpty() && types.stream().allMatch(DictModel::isDict))
					filled = DictModel.unpackInto(analysis, state, dict, iterable, st, true);
				else {
					Satisfiability it = Iteration.iterable(analysis, state, iterable, st);
					filled = state.bottomExecution();
					if (it != Satisfiability.NOT_SATISFIED)
						filled = pairs(analysis, state, dict, iterable, raising);
					if (types.stream().anyMatch(DictModel::isDict))
						// a dictionary, or another iterable
						filled = filled.lub(DictModel.unpackInto(analysis, state, dict, iterable, st, true));
					if (it != Satisfiability.SATISFIED)
						raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
				}
			}
			if (!missing(kwargs))
				// the keyword arguments override the entries of iterable: the
				// keys they surely have are tracked, as for a new dictionary
				filled = DictModel.unpackInto(analysis, filled, dict, kwargs, st, true);
			result = result.lub(compute(analysis, filled, dict));
		}
		return raising.moveFrom(result);
	}

	// adds the pairs of the iterable: none of them, or any of them, any number
	// of times
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> pairs(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			SymbolicExpression iterable,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Expression[] subs = getSubExpressions();
		// the elements are computed in the state where the dictionary has
		// been created, as they might be new objects (e.g. the tuples of zip)
		AnalysisState<A> elements = Iteration.elements(analysis, state, subs.length > 0 ? subs[0] : null, iterable,
				st, getMetaVariables(), raising);
		// no element
		AnalysisState<A> current = elements;
		boolean unknown = false, wrongLength = false;
		for (SymbolicExpression pair : elements.getExecutionExpressions()) {
			Set<Type> types = analysis.getRuntimeTypesOf(elements, pair, st);
			Type type = types.size() == 1 ? types.iterator().next() : null;
			Integer length = type != null && (Unpacking.isTuple(type) || ListModel.isList(type))
					? ListModel.length(analysis, elements, pair, st)
					: null;
			if (length == null) {
				unknown = true;
				current = current.lub(DictModel.anyEntry(analysis, current, dict, new PushAny(Untyped.INSTANCE, loc),
						new PushAny(Untyped.INSTANCE, loc), st));
			} else if (length != 2)
				wrongLength = true;
			else {
				HeapDereference deref = new HeapDereference(type.asPointerType().getInnerType(), pair, loc);
				AnalysisState<A> keys = analysis.smallStepSemantics(current,
						new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, 0, loc), loc), st);
				AnalysisState<A> values = analysis.smallStepSemantics(current,
						new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, 1, loc), loc), st);
				AnalysisState<A> next = current;
				for (SymbolicExpression key : keys.getExecutionExpressions())
					for (SymbolicExpression value : values.getExecutionExpressions())
						next = next.lub(DictModel.write(analysis, current, dict, key, value, st));
				current = next;
			}
		}
		if (unknown)
			// e.g. a str, whose length is not known, or an object
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		if (unknown || wrongLength)
			raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
		return current;
	}
}
