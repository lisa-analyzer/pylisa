package it.unive.pylisa.libraries.svverifier;

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
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code _sv_verifier.nondet_float_range(a, b)}, that
 * is {@code random.uniform(a, b)} ({@code a + (b - a) * random()}): the result
 * is any {@code float} (also infinite or {@code nan}, e.g. if {@code a} is
 * infinite), if {@code a} and {@code b} are numbers ({@code int}, {@code bool}
 * or {@code float}); {@code TypeError} is raised otherwise.
 */
public class NondetFloatRange extends PyNative {

	protected NondetFloatRange(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "nondet_float_range", params);
	}

	public static NondetFloatRange build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NondetFloatRange(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Satisfiability typed = hasType(analysis, context, state, args[0], INT.or(Type::isNumericType))
				.and(hasType(analysis, context, state, args[1], INT.or(Type::isNumericType)));
		return typeChecked(analysis, context, state, typed, unknown(analysis, context, state, Float32Type.INSTANCE));
	}
}
