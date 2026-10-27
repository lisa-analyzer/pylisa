package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
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
import it.unive.pylisa.libraries.sets.SetModel;
import it.unive.pylisa.symbolic.SetConstant;
import it.unive.pylisa.symbolic.operators.SetAdd;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A set literal {@code {e0, ..., en}}: a new set in the heap with the given
 * elements (see {@link SetModel}). When pandas is used, the set is instead a
 * symbolic value, as expected by the dataframe analyses.
 */
public class SetCreation extends NaryExpression {

	public SetCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "set", values);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		if (!LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			// a set in the heap (see SetModel)
			List<AnalysisState<A>> elements = new ArrayList<>();
			for (ExpressionSet param : params)
				elements.add(state.withExecutionExpressions(param));
			AnalysisState<A> created = SetModel.create(interprocedural.getAnalysis(), state, this, loc, elements,
					getMetaVariables());
			// the values of the elements are consumed here
			for (Expression element : getSubExpressions())
				created = created.forgetIdentifiers(element.getMetaVariables(), this);
			return created;
		}
		// when pandas is used, sets are symbolic values, as expected by the
		// dataframe analyses
		SetConstant set = new SetConstant(loc);
		if (params.length == 0)
			return interprocedural.getAnalysis().smallStepSemantics(state, set, this);
		Type setType = PyClassType.lookup(LibrarySpecificationProvider.SET);
		BinaryOperator add = SetAdd.INSTANCE;

		Set<BinaryExpression> ws = new HashSet<>(), tmp = new HashSet<>();
		for (SymbolicExpression element : params[0])
			ws.add(new BinaryExpression(setType, set, element, add, loc));
		for (int i = 1; i < params.length; i++) {
			tmp.addAll(ws);
			ws.clear();
			for (SymbolicExpression element : params[i])
				for (BinaryExpression setHead : tmp)
					ws.add(new BinaryExpression(setType, setHead, element, add, loc));
			tmp.clear();
		}

		AnalysisState<A> result = state.bottomExecution();
		for (BinaryExpression completeSet : ws)
			result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state, completeSet, this));

		return result;
	}

}
