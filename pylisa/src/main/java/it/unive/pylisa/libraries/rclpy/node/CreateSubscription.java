package it.unive.pylisa.libraries.rclpy.node;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.ros.lisa.symbolic.operators.ros.ROSTopicNameExpansion;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class CreateSubscription extends NaryExpression implements PluggableStatement {
	protected Statement st;

	protected CreateSubscription(CFG cfg, CodeLocation location, String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	public static CreateSubscription build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new CreateSubscription(cfg, location, "create_subscription", exprs);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		Set<SymbolicExpression> exprSet = new HashSet<>();
		// [AIG] self.namespace
		AccessInstanceGlobal aigNS = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0],
				"namespace");
		// compute semantics
		AnalysisState<A, H, V, T> aigSemanticsNS = aigNS.semantics(state, interprocedural, expressions);

		// [AIG] self.node_name
		AccessInstanceGlobal aigNodeName = new AccessInstanceGlobal(st.getCFG(), getLocation(), getSubExpressions()[0],
				"node_name");
		// compute semantics
		AnalysisState<A, H, V, T> aigSemanticsNodeName = aigNodeName.semantics(aigSemanticsNS, interprocedural,
				expressions);

		for (SymbolicExpression s : params[2]) {
			for (SymbolicExpression eNS : aigSemanticsNS.getComputedExpressions()) {
				ExpressionSet<ValueExpression> vesNS = state.getState().getHeapState().rewrite(eNS, st);
				for (SymbolicExpression eNodeName : aigSemanticsNodeName.getComputedExpressions()) {
					ExpressionSet<ValueExpression> vesNodeName = state.getState().getHeapState().rewrite(eNodeName, st);
					for (ValueExpression veNS : vesNS) {
						for (ValueExpression veNodeName : vesNodeName)
							exprSet.add(new TernaryExpression(StringType.INSTANCE, s, veNS, veNodeName,
									new ROSTopicNameExpansion(), getLocation()));
					}
				}
			}
		}

		params[2] = new ExpressionSet<>(exprSet);

		PyClassType subscriptionClassType = PyClassType.lookup(LibrarySpecificationProvider.RCLPY_SUBSCRIPTION);

		PyNewObj subscriptionObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(), "__init__",
				subscriptionClassType, Arrays.copyOfRange(getSubExpressions(), 1, getSubExpressions().length));
		subscriptionObj.setOffset(st.getOffset());
		AnalysisState<A, H, V, T> newSubscriptionAS = subscriptionObj.expressionSemantics(interprocedural,
				aigSemanticsNodeName, Arrays.copyOfRange(params, 1, params.length), expressions);
		state = aigSemanticsNodeName.lub(newSubscriptionAS);
		return state;
	}

	@Override
	public void setOriginatingStatement(Statement st) {
		this.st = st;
	}
}