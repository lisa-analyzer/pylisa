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
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.conversions.ConversionRaises;
import it.unive.pylisa.symbolic.operators.conversions.ToFloat;

/**
 * Native implementation of {@code float(x=None)}: {@code float()} is
 * {@code 0.0}; {@code x} must be a number or a {@code str} (otherwise
 * {@code TypeError} is raised). Invalid strings raise {@code ValueError}.
 */
public class FloatBuiltin extends PyNative {

	protected FloatBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "float", params);
	}

	public static FloatBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new FloatBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression x = args[0];
		if (x instanceof PyNoneConstant)
			return compute(analysis, state, new Constant(Float32Type.INSTANCE, 0f, loc));

		Satisfiability typed = hasType(analysis, state, x, STR.or(INT.or(t -> t.isNumericType())));
		UnaryExpression value = new UnaryExpression(Float32Type.INSTANCE, x, ToFloat.INSTANCE, loc);
		BinaryExpression raises = new BinaryExpression(BoolType.INSTANCE, x, new PyNoneConstant(loc),
				ConversionRaises.FLOAT, loc);
		return typeChecked(analysis, state, typed, ExceptionGuard.guardedCompute(analysis, state, raises,
				LibrarySpecificationProvider.VALUE_ERROR, value, getCFG(), loc, getOriginatingStatement(), this));
	}
}
