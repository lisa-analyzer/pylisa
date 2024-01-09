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
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.rclpy.subscription.ROSSubscriptionCallback;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.*;

public class Subscription extends TopicUser{
	private ROSSubscriptionCallback callbackFunction;

	private ROSLisaAnalysis rosLisaAnalysis;
	public Subscription(
			Node node,
			Topic topic,
			String msgType,
			ROSSubscriptionCallback callbackFunction) {
		super(node, topic, msgType);
		this.callbackFunction = callbackFunction;
	}


	public Subscription(String containerID, Topic topic, String msgType, Statement subscriptionStmt, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		super(containerID, topic, msgType);
		this.rosLisaAnalysis = new ROSLisaAnalysis(expr,subscriptionStmt, analysisState);
	}

	public Subscription(String containerID, Topic channel, String msgType, ROSSubscriptionCallback callbackFunction, Statement subscriptionStmt, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
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
		SymbolicExpression nodeExpr = ((Node)getContainer()).getLisaState().getSymbolicExpression();
		Statement subscriptionStmt = rosLisaAnalysis.getStatement();
		if (nodeExpr instanceof HeapReference) {
			HeapDereference hderef = new HeapDereference(Untyped.INSTANCE, nodeExpr, nodeExpr.getCodeLocation());
			//AccessChild aChild = new AccessChild(Untyped.INSTANCE, hderef, new Variable())
			if (subscriptionStmt instanceof UnresolvedCall) {
				Expression callbackFunction = ((UnresolvedCall) subscriptionStmt).getSubExpressions()[4];
				ROSSubscriptionCallback subCallback = new ROSSubscriptionCallback(subscriptionStmt.getCFG(), (SourceCodeLocation) subscriptionStmt.getLocation(), callbackFunction);
				Node container = (Node)getContainer();
				ExpressionSet[] exprSet = new ExpressionSet[2];
				Constant msg = new Constant(Untyped.INSTANCE, message.getMessage(), SyntheticLocation.INSTANCE);
				exprSet[0] = new ExpressionSet(nodeExpr);
				exprSet[1] = new ExpressionSet(msg);
				subCallback.forwardSemanticsAux(container.getLisaState().interproceduralAnalysis, container.getLisaState().getAnalysisState(), exprSet, new StatementStore<>(container.getLisaState().getAnalysisState()));
				var x = 3;
			}
		}
		//callbackFunction.
		this.addProcessedEvent(message.getNetworkEvent());
		System.out.println("[" + this.getContainer().getName() + "::" + this.getChannel().getID() + "] RECEIVED [" + message.getMessage() + "] FROM: [" + message.getNetworkEvent().getInitiator().getContainer().getName() + "]");
	}

	@Override
	public String getID() {
		return rosLisaAnalysis.getSymbolicExpression().getCodeLocation().toString();
	}

	@Override
	public NetworkEvent createNetworkEvent(NetworkMessage networkMessage) {
		return null;
	}

}