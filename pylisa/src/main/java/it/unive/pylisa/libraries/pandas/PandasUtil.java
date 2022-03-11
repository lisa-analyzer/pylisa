package it.unive.pylisa.libraries.pandas;

import java.util.HashSet;
import java.util.Set;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;

public class PandasUtil {

	private PandasUtil() {
	}

	public static <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> Set<Identifier> cleanUp(SymbolicExpression id, AnalysisState<A, H, V, T> state,
					ProgramPoint st)
					throws SemanticException {
		Set<Identifier> rewritten = new HashSet<>();
		@SuppressWarnings("unchecked")
		ExpressionSet<ValueExpression> tmp = state.getState().getDomainInstance(HeapDomain.class).rewrite(id, st);
		tmp.elements()
				.stream()
				.filter(Identifier.class::isInstance)
				.map(Identifier.class::cast)
				.forEach(rewritten::add);

		return rewritten;
	}
}
