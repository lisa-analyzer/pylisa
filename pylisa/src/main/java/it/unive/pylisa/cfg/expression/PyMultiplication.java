package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.numeric.Multiplication;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyLibraryUnitType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.StringMult;

public class PyMultiplication extends Multiplication {

	public PyMultiplication(CFG cfg, CodeLocation location, Expression left, Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> binarySemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					SymbolicExpression left,
					SymbolicExpression right,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		TypeSystem types = getProgram().getTypes();
		if (left.getRuntimeTypes(types).stream()
				.anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true))
				|| right.getRuntimeTypes(types).stream()
						.anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true)))
			// we allow scalar multiplication, but with no explicit handling for
			// now
			return state;

		// string repeat: STRING * Integer || Integer * String
		if ((left.getRuntimeTypes(types).stream().anyMatch(Type::isStringType)
				&& right.getRuntimeTypes(types).stream().anyMatch(Type::isNumericType)) ||
				(right.getRuntimeTypes(types).stream().anyMatch(Type::isStringType)
						&& left.getRuntimeTypes(types).stream().anyMatch(Type::isNumericType))) {

			return state.smallStepSemantics(
					new BinaryExpression(
							getStaticType(),
							left,
							right,
							StringMult.INSTANCE,
							getLocation()),
					this);
		}
		return super.binarySemantics(interprocedural, state, left, right, expressions);
	}
}
