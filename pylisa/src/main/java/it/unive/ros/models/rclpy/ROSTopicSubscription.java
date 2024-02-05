package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.rclpy.subscription.ROSSubscriptionCallback;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.*;

public class ROSTopicSubscription extends ROSTopicBasedNetworkEntity {
	private ROSSubscriptionCallback callbackFunction;

	public ROSTopicSubscription(
			ROSNode node,
			ROSTopic topic,
			String msgType,
			ROSSubscriptionCallback callbackFunction) {
		super(node, (it.unive.ros.models.rclpy.ROSTopic) topic, msgType);
		this.callbackFunction = callbackFunction;
	}


	public ROSTopicSubscription(String containerID, ROSTopic topic, String msgType, Statement subscriptionStmt, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		super(containerID, topic, msgType, expr, subscriptionStmt, analysisState);
	}

	public ROSTopicSubscription(String containerID, ROSTopic channel, String msgType, ROSSubscriptionCallback callbackFunction, Statement subscriptionStmt, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		this(containerID, channel, msgType, subscriptionStmt, expr, analysisState);
		this.callbackFunction = callbackFunction;
	}

	public ROSSubscriptionCallback getCallbackFunction() {
		return callbackFunction;
	}

    @Override
	public NetworkEntityType getNetworkEntityType() {
		return NetworkEntityType.READER;
	}

	@Override
	public void processMessage(NetworkMessage message) throws SemanticException {
		SymbolicExpression nodeExpr = ((ROSNode)getContainer()).getLisaState().getSymbolicExpression();
		Statement subscriptionStmt = getRosLisaAnalysis().getStatement();
		if (nodeExpr instanceof HeapReference) {
			HeapDereference hderef = new HeapDereference(Untyped.INSTANCE, nodeExpr, nodeExpr.getCodeLocation());
			//AccessChild aChild = new AccessChild(Untyped.INSTANCE, hderef, new Variable())
			if (subscriptionStmt instanceof UnresolvedCall) {
				Expression callbackFunction = ((UnresolvedCall) subscriptionStmt).getSubExpressions()[4];
				ROSSubscriptionCallback subCallback = new ROSSubscriptionCallback(subscriptionStmt.getCFG(), (SourceCodeLocation) subscriptionStmt.getLocation(), callbackFunction);
				ROSNode container = (ROSNode)getContainer();
				ExpressionSet[] exprSet = new ExpressionSet[2];
				Constant msg = new Constant(Untyped.INSTANCE, message.getMessage(), SyntheticLocation.INSTANCE);
				exprSet[0] = new ExpressionSet(nodeExpr);
				exprSet[1] = new ExpressionSet(msg);
				// subCallback.forwardSemanticsAux(container.getLisaState().interproceduralAnalysis, container.getLisaState().getAnalysisState(), exprSet, new StatementStore<>(container.getLisaState().getAnalysisState())); WON'T DO
				var x = 3;
			}
		}
		//callbackFunction.
		this.addProcessedEvent(message.getNetworkEvent());
		System.out.println("[" + this.getContainer().getName() + "::" + this.getChannel().getID() + "] RECEIVED [" + message.getMessage() + "] FROM: [" + message.getNetworkEvent().getInitiator().getContainer().getName() + "]");
	}

	@Override
	public String getID() {
		return getRosLisaAnalysis().getSymbolicExpression().getCodeLocation().toString();
	}

	@Override
	public NetworkEvent createNetworkEvent(NetworkMessage networkMessage) {
		return null;
	}

}