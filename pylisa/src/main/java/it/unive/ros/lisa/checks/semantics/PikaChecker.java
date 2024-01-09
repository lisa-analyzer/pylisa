package it.unive.ros.lisa.checks.semantics;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.models.pika.*;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.network.Network;
import it.unive.ros.network.NetworkChannel;
import it.unive.ros.network.NetworkEntityContainer;
import it.unive.ros.network.NetworkMessage;

public class PikaChecker implements
        SemanticCheck<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> {
    PikaNetwork pikaNetwork;
    public PikaChecker(PikaNetwork n) {
        this.pikaNetwork = n;
    }

    @Override
    public void beforeExecution(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool) {
    }

    @Override
    public void afterExecution(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool) {
    }

    @Override
    public boolean visitUnit(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
            Unit unit) {
        return true;
    }

    @Override
    public void visitGlobal(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
            Unit unit,
            Global global,
            boolean instance) {

    }

    @Override
    public boolean visit(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
            CFG graph) {

        return true;
    }

    @Override
    public boolean visit(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
            CFG graph,
            Statement node) {
        var results = tool.getResultOf(graph);
        for (AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> result : results) {
            try {
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("create_channel")) {
                    PikaConnection connection = new PikaConnection(node.getLocation().toString(), node.getLocation().toString());
                    if (pikaNetwork.getNetworkEntityContainer(node.getLocation().toString()) == null) {
                        pikaNetwork.addEntityContainer(connection);
                    }
                }
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("exchange_declare")) {

                    UnresolvedCall call = (UnresolvedCall) node;

                    String channelName = call.getSubExpressions()[1].toString().substring(1, call.getSubExpressions()[1].toString().length() - 1);
                    String exchangeType = call.getSubExpressions()[2].toString().substring(1, call.getSubExpressions()[2].toString().length() - 1);
                    NetworkChannel channel = pikaNetwork.getNetworkChannel(channelName);
                    if (channel == null) {
                        channel = new PikaExchangeChannel(channelName, exchangeType);
                        pikaNetwork.addNetworkChannel(channel);
                    }

                }
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("basic_publish")) {
                    UnresolvedCall call = (UnresolvedCall) node;
                    VariableRef channelVarRef = (VariableRef) call.getSubExpressions()[0];
                    StringLiteral networkChannelID = (StringLiteral) call.getSubExpressions()[1];
                    StringLiteral routingKeyStrLit = (StringLiteral) call.getSubExpressions()[2];
                    StringLiteral messageStrLit = (StringLiteral) call.getSubExpressions()[3];
                    String channelID = networkChannelID.toString().substring(1, networkChannelID.toString().length()-1);
                    String message = messageStrLit.toString().substring(1, messageStrLit.toString().length()-1);
                    String routingKey = routingKeyStrLit.toString().substring(1, routingKeyStrLit.toString().length()-1);
                    NetworkChannel channel = pikaNetwork.getNetworkChannel(channelID);
                    if (channel == null) {
                        throw new Exception("The channel does not exists!");
                    }
                    AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = result.getAnalysisStateAfter(channelVarRef);
                    for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
                        ExpressionSet e = analysisState.getState().rewrite(expr, channelVarRef, analysisState.getState());
                        String networkEntityContainerID = e.iterator().next().getCodeLocation().toString();
                        NetworkEntityContainer nec = pikaNetwork.getNetworkEntityContainer(networkEntityContainerID);
                        PikaPublisher pp = new PikaPublisher(networkEntityContainerID, routingKey, channel);
                        pikaNetwork.addNetworkEntity(pp, networkEntityContainerID);
                        pikaNetwork.createNetworkEvent(new NetworkMessage(message), pp);
                    }
                }
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("queue_declare")) {
                    UnresolvedCall call = (UnresolvedCall) node;
                    String queueName = call.getSubExpressions()[1].toString().substring(1, call.getSubExpressions()[1].toString().length() - 1);
                    NetworkChannel channel = pikaNetwork.getNetworkChannel(queueName);
                    if (channel == null || ! (channel instanceof PikaQueueChannel)) {
                        channel = new PikaQueueChannel(queueName);
                        pikaNetwork.addNetworkChannel(channel);
                    }
                }
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("queue_bind")) {
                    UnresolvedCall call = (UnresolvedCall) node;
                    String queueName = call.getSubExpressions()[2].toString().substring(1, call.getSubExpressions()[2].toString().length() - 1);
                    String exchangeName = call.getSubExpressions()[1].toString().substring(1, call.getSubExpressions()[1].toString().length() - 1);
                    String routingKey = call.getSubExpressions()[3].toString().substring(1, call.getSubExpressions()[3].toString().length() - 1);

                    NetworkChannel channel = pikaNetwork.getNetworkChannel(queueName);
                    if (!(channel instanceof PikaQueueChannel)) {
                        channel = new PikaQueueChannel(queueName);
                        pikaNetwork.addNetworkChannel(channel);
                    }
                    ((PikaExchangeChannel) pikaNetwork.getNetworkChannel(exchangeName)).bindQueue((PikaQueueChannel) channel, routingKey);
                }
                if (node instanceof UnresolvedCall && ((UnresolvedCall) node).getTargetName().equals("basic_consume")) {
                    UnresolvedCall call = (UnresolvedCall) node;
                    VariableRef channelVarRef = (VariableRef) call.getSubExpressions()[0];
                    StringLiteral networkChannelID = (StringLiteral) call.getSubExpressions()[2];
                    String channelID = networkChannelID.toString().substring(1, networkChannelID.toString().length()-1);
                    NetworkChannel channel = pikaNetwork.getNetworkChannel(channelID);
                    if (channel == null) {
                        throw new Exception("The channel does not exists!");
                    }
                    AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = result.getAnalysisStateAfter(channelVarRef);
                    for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
                        ExpressionSet e = analysisState.getState().rewrite(expr, channelVarRef, analysisState.getState());
                        String networkEntityContainerID = e.iterator().next().getCodeLocation().toString();
                        NetworkEntityContainer nec = pikaNetwork.getNetworkEntityContainer(networkEntityContainerID);
                        PikaConsumer pc = new PikaConsumer(networkEntityContainerID, channel);
                        pikaNetwork.addNetworkEntity(pc, networkEntityContainerID);
                    }
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        return true;
    }

    @Override
    public boolean visit(
            CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
            CFG graph,
            Edge edge) {
        return true;
    }


    public Network getNetwork() { return pikaNetwork; }
}
