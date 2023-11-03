package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.operators.DictPut;
import java.util.HashSet;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

public class DictionaryCreation extends NaryExpression {

	@SafeVarargs
	public DictionaryCreation(
			CFG cfg,
			CodeLocation loc,
			Pair<Expression, Expression>... values) {
		super(cfg, loc, "dict", toFlatArray(values));
	}

	private static Expression[] toFlatArray(
			Pair<Expression, Expression>[] values) {
		Expression[] result = new Expression[values.length * 2];
		for (int i = 0; i < values.length; i++) {
			int idx = 2 * i;
			result[idx] = values[i].getLeft();
			result[idx + 1] = values[i].getRight();
		}
		return result;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		CodeLocation loc = getLocation();
		DictConstant dict = new DictConstant(loc);

		if (params.length == 0)
			return state.smallStepSemantics(dict, this);

		Type dicttype = PyClassType.lookup(LibrarySpecificationProvider.DICT);
		TernaryOperator append = DictPut.INSTANCE;

		Set<TernaryExpression> ws = new HashSet<>(), tmp = new HashSet<>();
		for (SymbolicExpression firstkey : params[0])
			for (SymbolicExpression firstvalue : params[1])
				ws.add(new TernaryExpression(dicttype, dict, firstkey, firstvalue, append, loc));

		for (int i = 2; i < params.length - 1; i += 2) {
			ExpressionSet<SymbolicExpression> key = params[i];
			ExpressionSet<SymbolicExpression> value = params[i + 1];

			tmp.addAll(ws);
			ws.clear();
			for (TernaryExpression dicthead : tmp)
				for (SymbolicExpression field : key)
					for (SymbolicExpression init : value)
						ws.add(new TernaryExpression(dicttype, dicthead, field, init, append, loc));
			tmp.clear();
		}

		AnalysisState<A, H, V, T> result = state.bottom();
		for (TernaryExpression completedict : ws)
			result = result.lub(state.smallStepSemantics(completedict, this));

		return result;
	}

}
