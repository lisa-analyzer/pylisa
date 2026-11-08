package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import java.util.Collection;

/**
 * The length of a container that the analysis does not compute: any
 * non-negative integer.
 */
public final class UnknownLength {

	private UnknownLength() {
	}

	/**
	 * Yields a length that is not known: the computed expression is a variable
	 * holding any non-negative integer.
	 *
	 * @param <A>      the kind of abstract lattice
	 * @param <D>      the kind of abstract domain
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the current state
	 * @param pp       the program point computing the length
	 * @param metas    where the variable holding the length is added, that
	 *                     {@code pp} must forget
	 *
	 * @return the state whose computed expression is the length
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> of(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			ProgramPoint pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Variable length = new Variable(Int32Type.INSTANCE, "$len@" + loc.getCodeLocation(), loc);
		AnalysisState<A> result = analysis.assign(state, length, new PushAny(Int32Type.INSTANCE, loc), context, pp);
		result = analysis.assume(result, new BinaryExpression(BoolType.INSTANCE, length,
				new Constant(Int32Type.INSTANCE, 0, loc), ComparisonGe.INSTANCE, loc), context, pp, pp);
		metas.add(length);
		return result.withExecutionExpression(length);
	}
}
