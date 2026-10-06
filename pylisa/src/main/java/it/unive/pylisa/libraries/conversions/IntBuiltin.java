package it.unive.pylisa.libraries.conversions;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.conversions.ConversionRaises;
import it.unive.pylisa.symbolic.operators.conversions.ToInt;

/**
 * Native implementation of {@code int(x=None, base=None)}: {@code int()} is
 * {@code 0}; {@code x} can be a number (truncated towards zero) or, if
 * {@code base} is given, must be a {@code str} (otherwise {@code TypeError} is
 * raised). Invalid strings raise {@code ValueError}. An explicit {@code None}
 * is treated as an omitted argument.
 */
public class IntBuiltin extends PyNative {

	protected IntBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "int", params);
	}

	public static IntBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new IntBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression x = args[0], base = args[1];
		if (x instanceof PyNoneConstant && base instanceof PyNoneConstant)
			return compute(analysis, state, new Constant(Int32Type.INSTANCE, 0, loc));

		boolean noBase = base instanceof PyNoneConstant;
		Satisfiability typed = noBase ? hasType(analysis, state, x, STR.or(INT.or(t -> t.isNumericType())))
				: hasType(analysis, state, x, STR).and(hasType(analysis, state, base, INT));
		BinaryExpression value = new BinaryExpression(Int32Type.INSTANCE, x, base, ToInt.INSTANCE, loc);
		BinaryExpression raises = new BinaryExpression(BoolType.INSTANCE, x, base, ConversionRaises.INT, loc);
		return typeChecked(analysis, state, typed, ExceptionGuard.guardedCompute(analysis, state, raises,
				LibrarySpecificationProvider.VALUE_ERROR, value, getCFG(), loc, getOriginatingStatement(), this));
	}
}
