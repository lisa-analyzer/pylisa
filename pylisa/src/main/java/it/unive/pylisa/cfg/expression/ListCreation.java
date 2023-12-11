package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.operators.ListAppend;
import java.util.HashSet;
import java.util.Set;

public class ListCreation extends NaryExpression {

	public ListCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "list", values);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		ListConstant list = new ListConstant(loc);

		if (params.length == 0)
			return state.smallStepSemantics(list, this);

		Type listtype = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		BinaryOperator append = ListAppend.INSTANCE;

		Set<BinaryExpression> ws = new HashSet<>(), tmp = new HashSet<>();
		for (SymbolicExpression firstelement : params[0])
			ws.add(new BinaryExpression(listtype, list, firstelement, append, loc));

		for (int i = 1; i < params.length; i++) {
			tmp.addAll(ws);
			ws.clear();
			for (SymbolicExpression element : params[i])
				for (BinaryExpression listhead : tmp)
					ws.add(new BinaryExpression(listtype, listhead, element, append, loc));
			tmp.clear();
		}

		AnalysisState<A> result = state.bottom();
		for (BinaryExpression completelist : ws)
			result = result.lub(state.smallStepSemantics(completelist, this));

		return result;
	}
}