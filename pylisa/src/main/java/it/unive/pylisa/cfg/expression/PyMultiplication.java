package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.numeric.Multiplication;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyLibraryUnitType;
import it.unive.pylisa.symbolic.operators.StringMult;
import java.util.Set;

public class PyMultiplication extends Multiplication {

	public PyMultiplication(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type> rtsl = interprocedural.getAnalysis().getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = interprocedural.getAnalysis().getRuntimeTypesOf(state, right, this);

		if (rtsl.stream().anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true))
				|| rtsr.stream().anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true)))
			// we allow scalar multiplication, but with no explicit handling for
			// now
			return state;

		// string repeat: STRING * Integer || Integer * String
		if ((rtsl.stream().anyMatch(Type::isStringType) && rtsr.stream().anyMatch(Type::isNumericType)) ||
				(rtsr.stream().anyMatch(Type::isStringType) && rtsl.stream().anyMatch(Type::isNumericType))) {
			return interprocedural.getAnalysis().smallStepSemantics(
					state,
					new BinaryExpression(
							getStaticType(),
							left,
							right,
							StringMult.INSTANCE,
							getLocation()),
					this);
		}
		return super.fwdBinarySemantics(interprocedural, state, left, right, expressions);

	}
}
