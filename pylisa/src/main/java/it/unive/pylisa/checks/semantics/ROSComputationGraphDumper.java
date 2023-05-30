package it.unive.pylisa.checks.semantics;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.models.rclpy.*;

import java.io.IOException;
import java.io.Writer;
import java.lang.annotation.Native;
import java.util.Collection;
import java.util.Objects;

public class ROSComputationGraphDumper implements SemanticCheck<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> {

    RosComputationalGraph rosGraph = new RosComputationalGraph();

    @Override
    public void beforeExecution(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool) {
    }

    @Override
    public void afterExecution(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool) {
        StringBuilder dotGraph = new StringBuilder("digraph rosgraph {graph [pad=\"0.5\", nodesep=\"1\", ranksep=\"2\"];");
        for(Node n : rosGraph.getNodes()) {
            dotGraph.append(n.getName()).append("[shape=box];");
        }
        for (Topic t : rosGraph.getTopics()) {
            dotGraph.append(t.getName()).append(";");
        }
        for(Node n : rosGraph.getNodes()) {
            for (Publisher p : n.getPublishers()) {
                dotGraph.append(n.getName()).append(" -> ").append(p.getTopic().getName());
                dotGraph.append("[label=\"").append(p.getMsgType()).append("\"];");
            }
            for (Subscription s : n.getSubscribers()) {
                dotGraph.append(s.getTopic().getName()).append(" -> ").append(n.getName());
                dotGraph.append("[label=\"").append(s.getMsgType()).append(", ").append(s.getCallbackFunction()).append("\"];");
            }
        }
        dotGraph.append("}");
        tool.warn(dotGraph.toString());
        FileManager.WriteAction w = new FileManager.WriteAction() {
            @Override
            public void perform(Writer writer) throws IOException {
                writer.write(dotGraph.toString());
            }
        };
        try {
            tool.getFileManager().mkDotFile("test", w);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public boolean visitUnit(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool, Unit unit) {
        return true;
    }

    @Override
    public void visitGlobal(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool, Unit unit, Global global, boolean instance) {

    }

    @Override
    public boolean visit(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool, CFG graph) {

        return true;
    }

    @Override
    public boolean visit(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool, CFG graph, Statement node) {
        Collection<
                AnalyzedCFG<
                        SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                TypeEnvironment<InferredTypes>>,
                        PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                        TypeEnvironment<InferredTypes>>> results = tool.getResultOf(graph);
        for (AnalyzedCFG<
                SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                        TypeEnvironment<InferredTypes>>,
                PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                TypeEnvironment<InferredTypes>> result : results) {
                if (node instanceof UnresolvedCall) {
                    try {
                        Call c = tool.getResolvedVersion((UnresolvedCall)node, result);
                        if (c instanceof NativeCall) {
                            CodeMember codeMember = ((NativeCall) c).getTargets().iterator().next();
                            if (codeMember instanceof NativeCFG) {
                                NativeCFG nativeCFG = (NativeCFG) codeMember;
                                if (nativeCFG.getDescriptor().getName().equals("__init__") && nativeCFG.getDescriptor().getUnit().getName().equals(LibrarySpecificationProvider.RCLPY_NODE)) {
                                    String name = "<undefined>";
                                    AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>>,
                                            PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>> analysisState = result.getAnalysisStateAfter(node);
                                    for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
                                        if (expr instanceof HeapAllocationSite && ((HeapAllocationSite) expr).getField().equals("node_name")) {
                                            //name = analysisState.getState().getValueState().eval(expr, node);
                                            name = analysisState.getState().getValueState().eval((ValueExpression)expr, node).toString();
                                        }
                                    }
                                    rosGraph.addNode(new Node(name, result.getId()));
                                }
                                if (nativeCFG.getDescriptor().getName().equals("create_subscription") && nativeCFG.getDescriptor().getUnit().getName().equals(LibrarySpecificationProvider.RCLPY_NODE)) {
                                    AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>>,
                                            PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>> analysisState = result.getAnalysisStateAfter(node);
                                    String topicName = "<undefined>";
                                    String msgType = "<undefined>";
                                    String callbackFunction = "<undefined>";
                                    for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
                                        if (expr instanceof HeapReference && ((HeapReference) expr).getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_SUBSCRIPTION)))) {
                                            Variable access = new Variable(Untyped.INSTANCE, "topic_name", expr.getCodeLocation());
                                            HeapAllocationSite has = new HeapAllocationSite(StringType.INSTANCE, expr.getCodeLocation().getCodeLocation(), access, false, expr.getCodeLocation());
                                            topicName = analysisState.getState().getValueState().eval(has, node).toString();
                                            access = new Variable(Untyped.INSTANCE, "msg_type", expr.getCodeLocation());
                                            has = new HeapAllocationSite(StringType.INSTANCE, expr.getCodeLocation().getCodeLocation(), access, false, expr.getCodeLocation());
                                            msgType = analysisState.getState().getValueState().eval(has, node).toString();
                                            access = new Variable(Untyped.INSTANCE, "callback_func", expr.getCodeLocation());
                                            has = new HeapAllocationSite(StringType.INSTANCE, expr.getCodeLocation().getCodeLocation(), access, false, expr.getCodeLocation());
                                            String callback = analysisState.getState().getValueState().eval(has, node).toString();
                                            if (callback.startsWith("\"")) {
                                                callback = new String(callback.toCharArray(), 1, callback.length()-2);
                                            }
                                            callbackFunction = callback;
                                        }
                                    }
                                    /*String topicName = "<undefined>";
                                    for (SymbolicExpression expr : result.getAnalysisStateAfter(((UnresolvedCall) node).getSubExpressions()[2]).getComputedExpressions()) {
                                        topicName = ((Constant) expr).getValue().toString();
                                    }
                                    String msgType = "<undefined>";
                                    for (SymbolicExpression expr : result.getAnalysisStateAfter(((UnresolvedCall) node).getSubExpressions()[1]).getComputedExpressions()) {
                                        msgType = ((Variable) expr).toString();
                                    }
                                    String callbackFunction = "<undefined>";
                                    for (SymbolicExpression expr : result.getAnalysisStateAfter(((UnresolvedCall) node).getSubExpressions()[3]).getComputedExpressions()) {
                                        callbackFunction = expr.toString();
                                    }*/
                                    Topic topic = rosGraph.addOrGetTopic(topicName);
                                    rosGraph.getNodeByScopeId(result.getId()).addNewSubscriber(topic, msgType, callbackFunction);
                                }
                                if (nativeCFG.getDescriptor().getName().equals("create_publisher") && nativeCFG.getDescriptor().getUnit().getName().equals(LibrarySpecificationProvider.RCLPY_NODE)) {
                                    AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>>,
                                            PointBasedHeap, ValueEnvironment<ConstantPropagation>,
                                            TypeEnvironment<InferredTypes>> analysisState = result.getAnalysisStateAfter(node);
                                    String topicName = "<undefined>";
                                    String msgType = "<undefined>";
                                    for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
                                        if (expr instanceof HeapReference && ((HeapReference) expr).getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER)))) {
                                            Variable access = new Variable(Untyped.INSTANCE, "topic_name", expr.getCodeLocation());
                                            HeapAllocationSite has = new HeapAllocationSite(StringType.INSTANCE, expr.getCodeLocation().getCodeLocation(), access, false, expr.getCodeLocation());
                                            topicName = analysisState.getState().getValueState().eval(has, node).toString();
                                            access = new Variable(Untyped.INSTANCE, "msg_type", expr.getCodeLocation());
                                            has = new HeapAllocationSite(StringType.INSTANCE, expr.getCodeLocation().getCodeLocation(), access, false, expr.getCodeLocation());
                                            msgType = analysisState.getState().getValueState().eval(has, node).toString();
                                            System.out.println(expr);
                                        }
                                    }

                                    /*for (SymbolicExpression expr : result.getAnalysisStateAfter(((UnresolvedCall) node).getSubExpressions()[1]).getComputedExpressions()) {
                                        msgType = ((Variable) expr).toString();
                                    }*/
                                    Topic topic = rosGraph.addOrGetTopic(topicName);
                                    rosGraph.getNodeByScopeId(result.getId()).addNewPublisher(topic, msgType);
                                }
                            }
                        }
                    } catch (SemanticException e) {
                        return true;
                    }

                }

                }
        return true;
    }

    @Override
    public boolean visit(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>, PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool, CFG graph, Edge edge) {
        return true;
    }

    private String getValue(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
            TypeEnvironment<InferredTypes>>,
            PointBasedHeap, ValueEnvironment<ConstantPropagation>,
            TypeEnvironment<InferredTypes>> analysisState, ValueExpression expression, Statement node) {

        return "";
    }
}

