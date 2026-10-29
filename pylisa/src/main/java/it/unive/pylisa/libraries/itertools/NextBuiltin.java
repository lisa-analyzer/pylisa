package it.unive.pylisa.libraries.itertools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.libraries.EnumerateBuiltin;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.MapBuiltin;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.RangeBuiltin;
import it.unive.pylisa.libraries.ZipBuiltin;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.dicts.DictViews;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.libraries.sets.SetModel;
import java.util.Set;

/**
 * Native implementation of {@code next(iterator[, default])}: any element of
 * the iterator (see {@link Iteration#elements}), for the iterators of
 * {@code itertools} (see {@link ItertoolsModel}) and the zip, enumerate and map
 * objects. As the number of their elements is not tracked,
 * {@code StopIteration} might be raised, unless the iterator is never exhausted
 * (e.g. {@code itertools.count()}) or {@code default} is given (that is then a
 * possible result). The values that are not iterators (e.g. lists, or ints)
 * raise {@code TypeError}; for any other value (e.g. an object, whose class
 * might define {@code __next__}), the result is not known, and any exception
 * might be raised.
 */
public class NextBuiltin extends PyNative {

	protected NextBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "next", params);
	}

	public static NextBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NextBuiltin(cfg, location, exprs);
	}

	private static boolean iterator(
			Type t) {
		return ItertoolsModel.isIterator(t) || ZipBuiltin.isZip(t) || EnumerateBuiltin.isEnumerate(t)
				|| MapBuiltin.isMap(t);
	}

	// whether the values of type t are not iterators: the values of the
	// builtin classes (e.g. lists, that are iterable but not iterators, or
	// ints), functions, and the objects of the classes of the program whose
	// methods are all known, that do not define __next__
	private static boolean notIterator(
			Type t) {
		if (ListModel.isList(t) || Unpacking.isTuple(t) || DictModel.isDict(t) || SetModel.isSet(t)
				|| DictViews.isView(t) || RangeBuiltin.isRange(t) || t.isCodeMemberType()
				|| (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t)))
			return true;
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& ClassBases.isProgramClass(t.asPointerType().getInnerType().asUnitType().getUnit())
				&& PyBinaryDispatch.fullyKnown(t)
				&& t.asPointerType().getInnerType().asUnitType().getUnit() instanceof CompilationUnit
				&& ((CompilationUnit) t.asPointerType().getInnerType().asUnitType().getUnit())
						.getInstanceCodeMembersByName("__next__", true).isEmpty();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression it : params[0]) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, it, context, st);
			if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st)) {
				result = result.lub(compute(analysis, context, state, new PushAny(Untyped.INSTANCE, loc)));
				raise(raising, state, LibrarySpecificationProvider.EXCEPTION);
				continue;
			}
			boolean iterators = false, finite = false, notIterators = false, unknown = false;
			for (Type t : types)
				if (iterator(t)) {
					iterators = true;
					finite |= !ItertoolsModel.isIterator(t)
							|| !ItertoolsModel.infinite(analysis, context, state, it, st);
				} else if (notIterator(t))
					notIterators = true;
				else
					// e.g. an object whose class defines __next__, that is
					// not called
					unknown = true;
			if (iterators)
				result = result.lub(
						Iteration.elements(interprocedural, state, null, it, st, getMetaVariables(), raising));
			if (unknown) {
				result = result.lub(compute(analysis, context, state, new PushAny(Untyped.INSTANCE, loc)));
				raise(raising, state, LibrarySpecificationProvider.EXCEPTION);
			}
			if (notIterators)
				raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
			if (iterators && finite)
				// the iterator might be exhausted
				for (SymbolicExpression def : params.length > 1 ? params[1] : new ExpressionSet())
					if (missing(def))
						raise(raising, state, LibrarySpecificationProvider.STOP_ITERATION);
					else
						result = result.lub(compute(analysis, context, state, def));
		}
		return raising.moveFrom(result);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// never invoked: forwardSemanticsAux computes the result
		throw new SemanticException("next is evaluated by forwardSemanticsAux");
	}
}
