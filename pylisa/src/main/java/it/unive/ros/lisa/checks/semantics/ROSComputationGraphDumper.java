package it.unive.ros.lisa.checks.semantics;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.Symbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapAllocationSite;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.ResolvedCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.rclpy.node.SemanticsHelpers;
import it.unive.pylisa.libraries.rclpy.subscription.ROSSubscriptionCallback;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.lisa.analysis.constants.ConstantPropagationDomain;
import it.unive.ros.models.rclpy.ROSActionChannel;
import it.unive.ros.models.rclpy.ROSActionClient;
import it.unive.ros.models.rclpy.ROSActionServer;
import it.unive.ros.models.rclpy.ROSCommunicationChannel;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.models.rclpy.ROSNetworkEntity;
import it.unive.ros.models.rclpy.ROSNode;
import it.unive.ros.models.rclpy.ROSServiceChannel;
import it.unive.ros.models.rclpy.ROSServiceClient;
import it.unive.ros.models.rclpy.ROSServiceServer;
import it.unive.ros.models.rclpy.ROSTopic;
import it.unive.ros.models.rclpy.ROSTopicPublisher;
import it.unive.ros.models.rclpy.ROSTopicSubscription;
import it.unive.ros.models.rclpy.RosComputationalGraph;
import it.unive.ros.network.NetworkMessage;
import java.util.Collection;

public class ROSComputationGraphDumper
		implements
		SemanticCheck<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
				TypeEnvironment<TypeSet>>,
				SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<TypeSet>>> {

	private RosComputationalGraph rosGraph;

	private ROSNetwork rosNetwork;

	private ScopeId currentNodeScopeId;

	public ROSComputationGraphDumper(
			RosComputationalGraph rosGraph,
			ROSNetwork n) {
		this.rosGraph = rosGraph;
		this.rosNetwork = n;
	}

	private static final ConstantPropagationDomain CP_DOMAIN = new ConstantPropagationDomain();

	private static final SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
			TypeEnvironment<TypeSet>> DOMAIN = new SimpleAbstractDomain<>(new PointBasedHeap(), CP_DOMAIN,
					new InferredTypes());

	private ConstantPropagation evalConstant(
			ValueExpression expression,
			it.unive.lisa.program.cfg.ProgramPoint pp,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState)
			throws SemanticException {
		SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
				TypeEnvironment<TypeSet>> state = analysisState.getExecutionState();
		SemanticOracle oracle = DOMAIN.makeOracle(state);
		return CP_DOMAIN.eval(state.valueState, expression, pp, oracle);
	}

	private ExpressionSet rewriteExpr(
			SymbolicExpression expression,
			it.unive.lisa.program.cfg.ProgramPoint pp,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState)
			throws SemanticException {
		SemanticOracle oracle = DOMAIN.makeOracle(analysisState.getExecutionState());
		return oracle.rewrite(expression, pp);
	}

	@Override
	public void beforeExecution(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool) {
	}

	@Override
	public void afterExecution(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool) {
	}

	@Override
	public boolean visitUnit(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			Unit unit,
			Global global,
			boolean instance) {

	}

	@Override
	public boolean visit(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			CFG graph) {

		return true;
	}

	public void visitActionClient(
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			Statement publisher,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String actionName = "<undefined>";
		String actionType = "<undefined>";

		Variable access = new Variable(
				Untyped.INSTANCE,
				"action_name",
				expr.getCodeLocation());
		HeapAllocationSite has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		actionName = evalConstant(has, publisher, analysisState)
				.toString();
		actionName = actionName.substring(1,
				actionName.length() - 1);
		access = new Variable(
				Untyped.INSTANCE,
				"action_type",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		actionType = evalConstant(has, publisher, analysisState)
				.toString();
		actionType = actionType.substring(1,
				actionType.length() - 1);
		actionType = getMessageType(actionType, analysisState);

		ROSActionChannel channel = rosNetwork.getActionChannel(actionName);
		if (channel == null) {
			channel = new ROSActionChannel(actionName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSActionClient server = new ROSActionClient(nodeExpr.getCodeLocation().toString(), channel, actionType, expr,
				publisher, analysisState);
		rosNetwork.addNetworkEntity(server, nodeExpr.getCodeLocation().toString());
	}

	public void visitActionServer(
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			Statement publisher,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String actionName = "<undefined>";
		String actionType = "<undefined>";

		Variable access = new Variable(
				Untyped.INSTANCE,
				"action_name",
				expr.getCodeLocation());
		HeapAllocationSite has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		actionName = evalConstant(has, publisher, analysisState)
				.toString();
		actionName = actionName.substring(1,
				actionName.length() - 1);
		access = new Variable(
				Untyped.INSTANCE,
				"action_type",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		actionType = evalConstant(has, publisher, analysisState)
				.toString();
		actionType = actionType.substring(1,
				actionType.length() - 1);
		actionType = getMessageType(actionType, analysisState);

		ROSActionChannel channel = rosNetwork.getActionChannel(actionName);
		if (channel == null) {
			channel = new ROSActionChannel(actionName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSActionServer server = new ROSActionServer(nodeExpr.getCodeLocation().toString(), channel, actionType, expr,
				publisher, analysisState);
		rosNetwork.addNetworkEntity(server, nodeExpr.getCodeLocation().toString());
	}

	public void visitNode(
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			Statement node,
			HeapExpression expr,
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool)
			throws Exception {
		String nodeName = "<undefined>";
		String namespace = "<undefined>";
		Boolean startParamService = true; // true by default
		Boolean enableRosout = true; // true by default
		Variable access = new Variable(Untyped.INSTANCE,
				"node_name",
				expr.getCodeLocation());
		HeapAllocationSite has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		nodeName = evalConstant(has, node, analysisState)
				.toString();
		nodeName = nodeName.substring(1,
				nodeName.length() - 1);
		access = new Variable(Untyped.INSTANCE,
				"namespace",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		namespace = evalConstant(has, node, analysisState)
				.toString();
		namespace = namespace.substring(1,
				namespace.length() - 1);

		access = new Variable(Untyped.INSTANCE,
				"start_parameter_services",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());

		try {
			startParamService = (Boolean) evalConstant(has, node, analysisState).getConstant();
		} catch (Exception e) {
		}
		access = new Variable(Untyped.INSTANCE,
				"enable_rosout",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		try {
			enableRosout = (Boolean) evalConstant(has, node, analysisState).getConstant();
		} catch (Exception e) {
		}
		ROSNode n = new ROSNode(nodeName, namespace, startParamService, enableRosout, node, expr, analysisState,
				tool.getConfiguration().interproceduralAnalysis);
		rosNetwork.addEntityContainer(n);
	}

	public SymbolicExpression getNodeHeapReference(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			Expression node)
			throws SemanticException {
		String nodeName = null;
		AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
				TypeEnvironment<TypeSet>>> nodeSemantics = analyzedCFG.getAnalysisStateAfter(node);
		HeapReference nodeHR = new HeapReference(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE),
				nodeSemantics.getExecutionExpressions().elements.iterator().next(), node.getLocation());
		HeapDereference nodeDeref = new HeapDereference(nodeHR.getExpression().getStaticType(), nodeHR,
				nodeSemantics.getExecutionExpressions().elements.iterator().next().getCodeLocation());
		ExpressionSet nodeHAS = rewriteExpr(nodeDeref,
				node, nodeSemantics);
		return nodeHAS.iterator().next();
	}

	public void visitPublisher(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			UnresolvedCall unresolvedCall,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String topicName = "<undefined>";
		String msgType = "<undefined>";

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
		topicName = evalConstant(has, unresolvedCall, analysisState)
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
		msgType = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		Boolean avoidNamespaceConventions = false;
		try {
			avoidNamespaceConventions = isAvoidRosNamespaceConventions(analyzedCFG, analysisState,
					unresolvedCall.getSubExpressions()[3]);
			if (avoidNamespaceConventions) {
				rosNetwork.getWarnings().add(
						"avoid_ros_namespace_conventions = True in Publisher " + expr.getCodeLocation().toString());
			}
		} catch (Exception e) {
			rosNetwork.getWarnings().add(
					"Fail to get  avoid_ros_namespace_conventions for Publisher " + expr.getCodeLocation().toString());
		}
		ROSTopic channel = new ROSTopic(topicName, false, avoidNamespaceConventions);
		rosNetwork.addNetworkChannel(channel);
		ROSTopicPublisher p = new ROSTopicPublisher(nodeExpr.getCodeLocation().toString(), channel, msgType,
				unresolvedCall, expr, analysisState);
		rosNetwork.addNetworkEntity(p, nodeExpr.getCodeLocation().toString());
	}

	public String getMessageType(
			String variableName,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState) {
		for (Symbol s : analysisState.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).getKeys()) {
			if (s instanceof QualifiedNameSymbol) {
				QualifiedNameSymbol qnss = (QualifiedNameSymbol) s;
				if (qnss.getName().equals(variableName)) {
					return qnss.getQualifier() + "." + qnss.getName();
				}
			}
		}
		return variableName;
	}

	public void visitServiceClient(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			UnresolvedCall unresolvedCall,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String serviceName = "<undefined>";
		String msgType = "<undefined>";

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
		serviceName = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		serviceName = serviceName.substring(1,
				serviceName.length() - 1);
		access = new Variable(Untyped.INSTANCE,
				"srv_type",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		msgType = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		Boolean avoidNamespaceConventions = false;
		try {
			Expression e = SemanticsHelpers.getNamedParameterExpr(unresolvedCall.getSubExpressions(), "qos_profile");
			if (e != null) {
				avoidNamespaceConventions = isAvoidRosNamespaceConventions(analyzedCFG, analysisState,
						unresolvedCall.getSubExpressions()[3]);
				if (avoidNamespaceConventions) {
					rosNetwork.getWarnings().add("avoid_ros_namespace_conventions = True in Service Client "
							+ expr.getCodeLocation().toString());
				}
			}
		} catch (Exception e) {
			rosNetwork.getWarnings().add("Fail to get  avoid_ros_namespace_conventions for Service Client "
					+ expr.getCodeLocation().toString());
		}
		ROSServiceChannel channel = new ROSServiceChannel(serviceName, false, avoidNamespaceConventions);
		rosNetwork.addNetworkChannel(channel);
		ROSServiceClient s = new ROSServiceClient(nodeExpr.getCodeLocation().toString(), channel, msgType,
				unresolvedCall, expr, analysisState);
		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
		// ROSTopicPublisher p = new
		// ROSTopicPublisher(nodeExpr.getCodeLocation().toString(), (ROSTopic)
		// channel, msgType, publisher, expr, analysisState);
		// rosNetwork.addNetworkEntity(p,
		// nodeExpr.getCodeLocation().toString());
	}

	public void visitServiceServer(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			UnresolvedCall unresolvedCall,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String serviceName = "<undefined>";
		String msgType = "<undefined>";

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
		serviceName = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		serviceName = serviceName.substring(1,
				serviceName.length() - 1);
		access = new Variable(Untyped.INSTANCE,
				"srv_type",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		msgType = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		Boolean avoidNamespaceConventions = false;
		try {
			Expression e = SemanticsHelpers.getNamedParameterExpr(unresolvedCall.getSubExpressions(), "qos_profile");
			if (e != null) {
				avoidNamespaceConventions = isAvoidRosNamespaceConventions(analyzedCFG, analysisState, e);
				if (avoidNamespaceConventions) {
					rosNetwork.getWarnings().add("avoid_ros_namespace_conventions = True in Service Server "
							+ expr.getCodeLocation().toString());
				}
			}

		} catch (Exception e) {
			rosNetwork.getWarnings().add("Fail to get  avoid_ros_namespace_conventions for Service Server "
					+ expr.getCodeLocation().toString());
		}
		ROSServiceChannel channel = new ROSServiceChannel(serviceName, false, avoidNamespaceConventions);
		rosNetwork.addNetworkChannel(channel);
		ROSServiceServer s = new ROSServiceServer(nodeExpr.getCodeLocation().toString(), channel, msgType,
				unresolvedCall, expr, analysisState);
		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
	}

	public void visitSubscriber(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			UnresolvedCall unresolvedCall,
			HeapExpression expr,
			SymbolicExpression nodeExpr)
			throws Exception {
		String topicName = "<undefined>";
		String msgType = "<undefined>";
		// String callbackFunction = "<undefined>";
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
		topicName = evalConstant(has, unresolvedCall, analysisState)
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
		msgType = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		access = new Variable(Untyped.INSTANCE,
				"callback",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());
		String callback = evalConstant(has, unresolvedCall, analysisState)
				.toString();
		// ROSSubscriptionCallback callbackFunction =
		// getROSCallbackFunction(((UnresolvedCall)publisher).getSubExpressions()[4]);
		if (callback.startsWith("\"")) {
			callback = new String(callback
					.toCharArray(),
					1,
					callback.length()
							- 2);
		}

		access = new Variable(Untyped.INSTANCE,
				"qos_profile",
				expr.getCodeLocation());
		has = new HeapAllocationSite(
				StringType.INSTANCE,
				expr.getCodeLocation()
						.getCodeLocation(),
				access, false,
				expr.getCodeLocation());

		// callbackFunction = callback;
		Boolean avoidNamespaceConventions = false;
		try {
			avoidNamespaceConventions = isAvoidRosNamespaceConventions(analyzedCFG, analysisState,
					unresolvedCall.getSubExpressions()[4]);
			if (avoidNamespaceConventions) {
				rosNetwork.getWarnings().add(
						"avoid_ros_namespace_conventions = True in Subscription " + expr.getCodeLocation().toString());
			}
		} catch (Exception e) {
			rosNetwork.getWarnings().add("Fail to get  avoid_ros_namespace_conventions for Subscription "
					+ expr.getCodeLocation().toString());
		}
		ROSCommunicationChannel channel = new ROSTopic(topicName, false, avoidNamespaceConventions);
		rosNetwork.addNetworkChannel(channel);
		ROSSubscriptionCallback callbackFunction = new ROSSubscriptionCallback(unresolvedCall.getCFG(),
				(SourceCodeLocation) unresolvedCall.getLocation(), unresolvedCall.getSubExpressions()[3]);
		ROSTopicSubscription s = new ROSTopicSubscription(nodeExpr.getCodeLocation().toString(), (ROSTopic) channel,
				msgType, callbackFunction, unresolvedCall, expr, analysisState);

		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
	}

	public void visitAnalyzedCFG(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			Statement node)
			throws Exception {
		if (node instanceof PyNewObj) {
			PyNewObj obj = (PyNewObj) node;
			Type staticType = obj.getStaticType();
			if (staticType instanceof PyClassType) {
				PyClassType pyCObjClassType = (PyClassType) staticType;
				if (pyCObjClassType.getUnit().getImmediateAncestors()
						.contains(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE).getUnit())) {
					// we are creating a rclpy Node.
					AnalysisState<
							SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
									TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
					// in the analysis state we have a rclpy Node.
					for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {
						if (expr instanceof HeapReference
						/*
						 * && expr.getStaticType().equals(new
						 * ReferenceType(PyClassType.lookup(
						 * LibrarySpecificationProvider.RCLPY_NODE)))
						 */) {
							visitNode(analysisState, node, (HeapExpression) expr, tool);
							return;
						}
					}
				}
			}
			if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONCLIENT))) {
				AnalysisState<
						SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
								TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
				for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {
					if (expr instanceof HeapReference
					/*
					 * && expr.getStaticType().equals(new
					 * ReferenceType(PyClassType.lookup(
					 * LibrarySpecificationProvider.RCLPY_NODE)))
					 */) {
						SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState,
								((PyNewObj) node).getSubExpressions()[0]);
						visitActionClient(analysisState, node, (HeapExpression) expr, nodeExpr);
						return;
					}
				}
			} else if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONSERVER))) {
				AnalysisState<
						SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
								TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
				for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {
					if (expr instanceof HeapReference
					/*
					 * && expr.getStaticType().equals(new
					 * ReferenceType(PyClassType.lookup(
					 * LibrarySpecificationProvider.RCLPY_NODE)))
					 */) {
						SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState,
								((PyNewObj) node).getSubExpressions()[0]);
						visitActionServer(analysisState, node, (HeapExpression) expr, nodeExpr);
						return;
					}
				}
			}
		} else if (node instanceof UnresolvedCall) {
			Call c = tool.getResolvedVersion((UnresolvedCall) node, analyzedCFG);
			if (c instanceof ResolvedCall) {
				Collection<CodeMember> targets = ((ResolvedCall) c).getTargets();
				if (targets.isEmpty()) {
					return;
				}
				CodeMember codeMember = targets.iterator()
						.next();
				if (codeMember instanceof NativeCFG) {
					NativeCFG nativeCFG = (NativeCFG) codeMember;
					visitNativeCFG(tool, nativeCFG, analyzedCFG, (UnresolvedCall) node);
				}
			}
		}
	}

	public Boolean isAvoidRosNamespaceConventions(
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			Expression qosProfile)
			throws SemanticException {
		AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
				TypeEnvironment<TypeSet>>> qosSemantics = analyzedCFG.getAnalysisStateAfter(qosProfile);
		HeapReference qosHR = new HeapReference(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE),
				qosSemantics.getExecutionExpressions().elements.iterator().next(), qosProfile.getLocation());
		HeapDereference qosDeref = new HeapDereference(qosHR.getExpression().getStaticType(), qosHR,
				qosSemantics.getExecutionExpressions().elements.iterator().next().getCodeLocation());
		ExpressionSet qosHAS = rewriteExpr(qosDeref,
				qosProfile, qosSemantics);
		SymbolicExpression e = qosHAS.iterator().next();
		if (e instanceof HeapAllocationSite) {

			// HeapAllocationSite has = (HeapAllocationSite) e;
			Variable access = new Variable(Untyped.INSTANCE,
					"avoid_ros_namespace_conventions",
					e.getCodeLocation());
			HeapAllocationSite has = new HeapAllocationSite(
					StringType.INSTANCE,
					e.getCodeLocation()
							.getCodeLocation(),
					access, false,
					e.getCodeLocation());
			return (Boolean) evalConstant(has, qosProfile, analysisState)
					.getConstant();
		}
		return false;
	}

	public void visitNativeCFG(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			NativeCFG nativeCFG,
			AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analyzedCFG,
			Statement node)
			throws Exception {
		String nativeCFGDescriptorName = nativeCFG.getDescriptor().getName();
		String nativeCFGDescriptorUnitName = nativeCFG.getDescriptor().getUnit().getName();
		System.out.println(" ! ! ! ! ! ! " + nativeCFGDescriptorName);
		if (node instanceof PyNewObj) {
			PyNewObj obj = (PyNewObj) node;
			Type staticType = obj.getStaticType();
			if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONCLIENT))) {
			} else if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONSERVER))) {
			}
		} else if ((nativeCFGDescriptorName.equals("__init__")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY_NODE))) {
			// Node.__init__( ... )
		} else if ((nativeCFGDescriptorName.equals("create_node")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY))) {
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
			for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {
				if (expr instanceof HeapReference
				/*
				 * && expr.getStaticType().equals(new
				 * ReferenceType(PyClassType.lookup(LibrarySpecificationProvider
				 * .RCLPY_NODE)))
				 */) {
					visitNode(analysisState, node, (HeapExpression) expr, tool);
					return;
				}
			}
			var x = 3;
		} else if (nativeCFG.getDescriptor().getName()
				.equals("create_subscription")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
								.getStaticType()
								.equals(new ReferenceType(
										PyClassType
												.lookup(LibrarySpecificationProvider.RCLPY_SUBSCRIPTION)))) {
					// AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>,
					// ValueEnvironment<ConstantPropagation>,
					// TypeEnvironment<TypeSet>>> nodeSemantics =
					// analyzedCFG.getAnalysisStateAfter(unresolvedCall.getSubExpressions()[0]);
					SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState,
							unresolvedCall.getSubExpressions()[0]);

					visitSubscriber(analyzedCFG, analysisState, unresolvedCall, (HeapExpression) expr, nodeExpr);
				}
			}
		} else if (nativeCFGDescriptorName
				.equals("create_publisher")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
								.getStaticType()
								.equals(new ReferenceType(
										PyClassType
												.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER)))) {
					// AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>,
					// ValueEnvironment<ConstantPropagation>,
					// TypeEnvironment<TypeSet>>> nodeSemantics =
					// analyzedCFG.getAnalysisStateAfter(unresolvedCall.getSubExpressions()[0]);
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState,
							unresolvedCall.getSubExpressions()[0]);
					visitPublisher(analyzedCFG, analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);

					// var aigSemanticsNodeName =
					// aigNodeName.forwardSemantics(nodeSemantics,
					// tool.getConfiguration().interproceduralAnalysis, new
					// StatementStore<>(nodeSemantics));
					// ConstantPropagation cp =
					// evalConstant((ValueExpression)
					// nodeSemantics.getExecutionExpressions().elements.iterator().next(),
					// node, analysisState);
					// compute semantics
					// AnalysisState<A> aigSemanticsNodeName =
					// aigNodeName.forwardSemantics(analysisState,
					// tool.getConfiguration().interproceduralAnalysis,
					// expressions);
					// Publisher p = visitPublisher();
				}
			}
		} else if (nativeCFGDescriptorName
				.equals("create_service")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
								.getStaticType()
								.equals(new ReferenceType(
										PyClassType
												.lookup(LibrarySpecificationProvider.RCLPY_SERVICE)))) {
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState,
							unresolvedCall.getSubExpressions()[0]);
					visitServiceServer(analyzedCFG, analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);
				}
			}
		} else if (nativeCFGDescriptorName.equals("create_client")
				&& nativeCFGDescriptorUnitName
						.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getExecutionExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
								.getStaticType()
								.equals(new ReferenceType(
										PyClassType
												.lookup(LibrarySpecificationProvider.RCLPY_CLIENT)))) {
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState,
							unresolvedCall.getSubExpressions()[0]);
					visitServiceClient(analyzedCFG, analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);
				}
			}
		} else if (nativeCFGDescriptorName.equals("publish")
				&& nativeCFGDescriptorUnitName.equals(LibrarySpecificationProvider.RCLPY_PUBLISHER)) {
			// get the Pubisher
			// generate NetworkEvent

			Expression publisherExpression = ((UnresolvedCall) node).getSubExpressions()[0];
			Expression message = ((UnresolvedCall) node).getSubExpressions()[1];
			AnalysisState<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<
									TypeSet>>> publisherSemantics = analyzedCFG
											.getAnalysisStateAfter(publisherExpression);

			HeapReference publisherHR = new HeapReference(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE),
					publisherSemantics.getExecutionExpressions().elements.iterator().next(),
					publisherExpression.getLocation());
			HeapDereference publisherDeref = new HeapDereference(publisherHR.getExpression().getStaticType(),
					publisherHR,
					publisherSemantics.getExecutionExpressions().elements.iterator().next().getCodeLocation());
			ExpressionSet publisherExprSet = rewriteExpr(publisherDeref,
					node, publisherSemantics);
			ROSNetworkEntity ne = rosNetwork
					.getNetworkEntity(publisherExprSet.iterator().next().getCodeLocation().toString());
			System.out.println(ne);
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> messageSemantics = analyzedCFG.getAnalysisStateAfter(message);
			Object _message = null;
			for (SymbolicExpression e : messageSemantics.getExecutionExpressions()) {
				ConstantPropagation cp = evalConstant((ValueExpression) e, message, messageSemantics);
				_message = cp.isTop() ? "#TOP#" : cp.getConstant();
			}
			// messageSemantics.getState().getValueState().eval(message,
			// message.getLocation());
			NetworkMessage NetworkMessage = new NetworkMessage(_message, ne.getType());
			// NetworkEvent event = ne.createNetworkEvent(message);
			rosNetwork.createNetworkEvent(NetworkMessage, ne);
		}
	}

	@Override
	public boolean visit(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			CFG graph,
			Statement node) {
		Collection<
				AnalyzedCFG<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<TypeSet>>>> results = tool
								.getResultOf(graph);
		try {
			for (AnalyzedCFG<
					SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> result : results) {
				visitAnalyzedCFG(tool, result, node);
				if (node instanceof UnresolvedCall) {

					Call c = tool.getResolvedVersion((UnresolvedCall) node, result);
					if (c instanceof ResolvedCall) {
						Collection<CodeMember> targets = ((ResolvedCall) c).getTargets();
						if (targets.isEmpty()) {
							continue;
						}
						CodeMember codeMember = targets.iterator()
								.next();
					}
				}
			}
		} catch (Exception e) {
			System.out.println(e.getMessage());
			return true;
		}
		return true;

	}

	@Override
	public boolean visit(
			SemanticTool<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>,
					SimpleAbstractDomain<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<TypeSet>>> tool,
			CFG graph,
			Edge edge) {
		if (graph.getDescriptor().getName().equals("$main")) {
			var nodeAnalysisState = tool.getResultOf(graph);
			// get the first result.
			var analyzedCFG = nodeAnalysisState.stream().iterator().next();
		}
		return true;
	}

	public RosComputationalGraph getRosGraph() {
		return rosGraph;
	}

	public ROSNetwork getNetwork() {
		return rosNetwork;
	}
}
