package it.unive.ros.lisa.checks.semantics;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.ResolvedCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.models.rclpy.Node;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.models.rclpy.Service;
import it.unive.ros.models.rclpy.Topic;
import java.util.Collection;

public class ROSComputationGraphDumper
		implements
		SemanticCheck<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<InferredTypes>>> {

	private RosComputationalGraph rosGraph;

	private ScopeId currentNodeScopeId;

	public ROSComputationGraphDumper(
			RosComputationalGraph rosGraph) {
		this.rosGraph = rosGraph;
	}

	@Override
	public void beforeExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool) {
	}

	@Override
	public void afterExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool) {
	}

	@Override
	public boolean visitUnit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool,
			Unit unit,
			Global global,
			boolean instance) {

	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool,
			CFG graph) {

		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Statement node) {
		Collection<AnalyzedCFG<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<InferredTypes>>>> results = tool
								.getResultOf(graph);
		for (AnalyzedCFG<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<InferredTypes>>> result : results) {
			if (node instanceof UnresolvedCall) {
				try {
					Call c = tool.getResolvedVersion((UnresolvedCall) node, result);
					if (c instanceof ResolvedCall) {
						Collection<CodeMember> targets = ((ResolvedCall) c).getTargets();
						if (targets.size() == 0) {
							continue;
						}
						CodeMember codeMember = targets.iterator()
								.next();
						if (codeMember instanceof NativeCFG) {
							NativeCFG nativeCFG = (NativeCFG) codeMember;
							if ((nativeCFG.getDescriptor().getName().equals("__init__")
									&& nativeCFG.getDescriptor()
											.getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY_NODE))) {
								String name = "<undefined>";
								String namespace = "";
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									if (expr instanceof HeapAllocationSite
											&& ((HeapAllocationSite) expr)
													.getField()
													.equals("node_name")) {
										// name =
										// analysisState.getState().getValueState().eval(expr,
										// node);
										name = analysisState.getState()
												.getValueState()
												.eval((ValueExpression) expr,
														node,
														analysisState.getState())
												.toString();
										name = name.substring(1,
												name.length() - 1);
									}
									if (expr instanceof HeapAllocationSite
											&& ((HeapAllocationSite) expr)
													.getField()
													.equals("namespace")) {
										namespace = analysisState.getState()
												.getValueState()
												.eval((ValueExpression) expr,
														node,
														analysisState.getState())
												.toString();
										if (namespace.length() > 0) {
											namespace = namespace.substring(
													1,
													namespace.length()
															- 1);
										}

									}
								}
								rosGraph.addNode(new Node(name, namespace,
										result.getId()));
								currentNodeScopeId = result.getId();
							}
							if ((nativeCFG.getDescriptor().getName().equals("create_node")
									&& nativeCFG.getDescriptor()
											.getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY))) {
								String name = "<undefined>";
								String namespace = "";
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									HeapReference ref = (HeapReference) expr;
									HeapDereference deref = new HeapDereference(ref
											.getExpression()
											.getStaticType(), expr,
											node.getLocation());
									AccessChild accessChild = new AccessChild(
											StringType.INSTANCE, deref,
											new Variable(StringType.INSTANCE,
													"node_name",
													node.getLocation()),
											node.getLocation());
									ExpressionSet nodeNameSet = analysisState
											.getState()
											.rewrite(accessChild,
													node,
													analysisState.getState());
									for (SymbolicExpression se : nodeNameSet) {
										if (se instanceof HeapAllocationSite) {
											name = analysisState.getState()
													.getValueState()
													.eval((ValueExpression) se,
															node,
															analysisState.getState())
													.toString();
											name = name.substring(1,
													name.length() - 1);
										}
									}
									accessChild = new AccessChild(
											StringType.INSTANCE, deref,
											new Variable(StringType.INSTANCE,
													"namespace",
													node.getLocation()),
											node.getLocation());
									ExpressionSet namespaceSet = analysisState
											.getState()
											.rewrite(accessChild,
													node,
													analysisState.getState());
									for (SymbolicExpression se : namespaceSet) {
										if (se instanceof HeapAllocationSite) {
											// node);
											namespace = analysisState
													.getState()
													.getValueState()
													.eval((ValueExpression) se,
															node,
															analysisState.getState())
													.toString();
											if (namespace.length() > 0) {
												namespace = name.substring(
														1,
														name.length() - 1);
											}
										}
									}
								}
								rosGraph.addNode(new Node(name, namespace,
										result.getId()));
								currentNodeScopeId = result.getId();
							}

							if (nativeCFG.getDescriptor().getName()
									.equals("create_subscription")
									&& nativeCFG.getDescriptor().getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								String topicName = "<undefined>";
								String msgType = "<undefined>";
								String callbackFunction = "<undefined>";
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									if (expr instanceof HeapReference
											&& ((HeapReference) expr)
													.getStaticType()
													.equals(new ReferenceType(
															PyClassType.lookup(
																	LibrarySpecificationProvider.RCLPY_SUBSCRIPTION)))) {
										Variable access = new Variable(
												Untyped.INSTANCE,
												"topic_name",
												expr.getCodeLocation());
										HeapAllocationSite has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										topicName = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										topicName = topicName.substring(1,
												topicName.length() - 1);
										access = new Variable(Untyped.INSTANCE,
												"msg_type",
												expr.getCodeLocation());
										has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										msgType = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										access = new Variable(Untyped.INSTANCE,
												"callback_func",
												expr.getCodeLocation());
										has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										String callback = analysisState
												.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										if (callback.startsWith("\"")) {
											callback = new String(callback
													.toCharArray(),
													1,
													callback.length()
															- 2);
										}
										for (SymbolicExpression e : result
												.getAnalysisStateAfter(
														((UnresolvedCall) node)
																.getSubExpressions()[1])
												.getComputedExpressions()) {
											msgType = ((Variable) e)
													.toString();
										}
										callbackFunction = callback;
									}
								}

								Topic topic = rosGraph.addOrGetTopic(topicName);
								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewSubscriber(topic, msgType,
												callbackFunction);
							}
							if (nativeCFG.getDescriptor().getName()
									.equals("create_publisher")
									&& nativeCFG.getDescriptor().getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								String topicName = "<undefined>";
								String msgType = "<undefined>";
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									if (expr instanceof HeapReference
											&& ((HeapReference) expr)
													.getStaticType()
													.equals(new ReferenceType(
															PyClassType
																	.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER)))) {
										Variable access = new Variable(
												Untyped.INSTANCE,
												"topic_name",
												expr.getCodeLocation());
										HeapAllocationSite has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										topicName = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										topicName = topicName.substring(1,
												topicName.length() - 1);
										access = new Variable(Untyped.INSTANCE,
												"msg_type",
												expr.getCodeLocation());
										has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										msgType = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										for (SymbolicExpression e : result
												.getAnalysisStateAfter(
														((UnresolvedCall) node)
																.getSubExpressions()[1])
												.getComputedExpressions()) {
											msgType = ((Variable) e)
													.toString();
										}
									}
								}

								Topic topic = rosGraph.addOrGetTopic(topicName);
								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewPublisher(topic, msgType);
							}
							if (nativeCFG.getDescriptor().getName().equals("create_service")
									&& nativeCFG.getDescriptor().getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								String topicName = "<undefined>";
								String msgType = "<undefined>";
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									if (expr instanceof HeapReference
											&& ((HeapReference) expr)
													.getStaticType()
													.equals(new ReferenceType(
															PyClassType.lookup(
																	LibrarySpecificationProvider.RCLPY_SERVICE)))) {
										Variable access = new Variable(
												Untyped.INSTANCE,
												"srv_name",
												expr.getCodeLocation());
										HeapAllocationSite has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										topicName = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										topicName = topicName.substring(1,
												topicName.length() - 1);
										access = new Variable(Untyped.INSTANCE,
												"srv_type",
												expr.getCodeLocation());
										has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										msgType = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										for (SymbolicExpression e : result
												.getAnalysisStateAfter(
														((UnresolvedCall) node)
																.getSubExpressions()[1])
												.getComputedExpressions()) {
											msgType = ((Variable) e)
													.toString();
										}
									}
								}
								rosGraph.getNodeByScopeId(result.getId())
										.addNewService(new Service(topicName,
												msgType, ""));
								Topic t = rosGraph.addOrGetTopic(topicName + "Reply");
								rosGraph.getNodeByScopeId(result.getId())
										.addNewPublisher(t, msgType);
								t = rosGraph.addOrGetTopic(topicName + "Request");
								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewSubscriber(t, msgType, "");
							}
							if (nativeCFG.getDescriptor().getName().equals("create_client")
									&& nativeCFG.getDescriptor()
											.getUnit().getName()
											.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
								AnalysisState<
										SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
												TypeEnvironment<InferredTypes>>> analysisState = result
														.getAnalysisStateAfter(node);
								String topicName = "<undefined>";
								String msgType = "<undefined>";
								for (SymbolicExpression expr : analysisState
										.getComputedExpressions()) {
									if (expr instanceof HeapReference
											&& ((HeapReference) expr)
													.getStaticType()
													.equals(new ReferenceType(
															PyClassType.lookup(
																	LibrarySpecificationProvider.RCLPY_CLIENT)))) {
										Variable access = new Variable(
												Untyped.INSTANCE,
												"srv_name",
												expr.getCodeLocation());
										HeapAllocationSite has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										topicName = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										topicName = topicName.substring(1,
												topicName.length() - 1);
										access = new Variable(Untyped.INSTANCE,
												"srv_type",
												expr.getCodeLocation());
										has = new HeapAllocationSite(
												StringType.INSTANCE,
												expr.getCodeLocation()
														.getCodeLocation(),
												access, false,
												expr.getCodeLocation());
										msgType = analysisState.getState()
												.getValueState()
												.eval(has, node, analysisState.getState())
												.toString();
										for (SymbolicExpression e : result
												.getAnalysisStateAfter(
														((UnresolvedCall) node)
																.getSubExpressions()[1])
												.getComputedExpressions()) {
											msgType = ((Variable) e)
													.toString();
										}
									}
								}

								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewService(new Service(topicName,
												msgType, ""));
								Topic t = rosGraph.addOrGetTopic(topicName + "Reply");
								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewSubscriber(t, msgType, "");
								t = rosGraph.addOrGetTopic(topicName + "Request");
								rosGraph.getNodeByScopeId(currentNodeScopeId)
										.addNewPublisher(t, msgType);
								// rosGraph.getNodeByScopeId(result.getId()).addNewServicePublisher(topic,
								// msgType);
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
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Edge edge) {
		return true;
	}

	public RosComputationalGraph getRosGraph() {
		return rosGraph;
	}
}
