package it.unive.pylisa.libraries.itertools;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.RangeBuiltin;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;

/**
 * Native implementation of the builtin {@code reversed(seq)}: an iterator over
 * the elements of {@code seq} (see {@link ItertoolsModel}), whose order is not
 * tracked. {@code TypeError} is raised if {@code seq} might not be a sequence:
 * lists, tuples, strings, bytes and ranges are, values that are not iterable
 * are not, and anything else (e.g. sets, iterators) is not decided. As in
 * python, the result has no length and cannot be indexed.
 */
public class ReversedBuiltin extends ItertoolBuiltin {

	protected ReversedBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "reversed", params);
	}

	public static ReversedBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReversedBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return checked(analysis, context, state, sequence(analysis.getRuntimeTypesOf(state, args[0], context, st)),
				ItertoolsModel.REVERSED,
				new String[] { ItertoolsModel.ITERABLE }, new SymbolicExpression[] { args[0] });
	}

	private Satisfiability sequence(
			Set<Type> types) {
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, st))
			return Satisfiability.UNKNOWN;
		boolean sequence = false, notIterable = false;
		for (Type t : types)
			if (ListModel.isList(t) || Unpacking.isTuple(t) || t.isStringType() || t instanceof PyBytesType
					|| RangeBuiltin.isRange(t))
				sequence = true;
			else if (t.isCodeMemberType()
					|| (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t)))
				notIterable = true;
			else
				return Satisfiability.UNKNOWN;
		if (sequence && notIterable)
			return Satisfiability.UNKNOWN;
		return sequence ? Satisfiability.SATISFIED : Satisfiability.NOT_SATISFIED;
	}
}
