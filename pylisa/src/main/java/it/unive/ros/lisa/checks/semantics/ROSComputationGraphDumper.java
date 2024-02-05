package it.unive.ros.lisa.checks.semantics;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.Symbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.checks.semantic.CheckToolWithAnalysisResults;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.interprocedural.ScopeId;
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
import it.unive.lisa.symbolic.heap.AccessChild;
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
import it.unive.pylisa.libraries.rclpy.subscription.ROSSubscriptionCallback;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.models.rclpy.*;
import it.unive.ros.models.rclpy.ROSNetwork;
import it.unive.ros.network.*;

import java.util.Collection;

public class ROSComputationGraphDumper
		implements
		SemanticCheck<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> {

	private RosComputationalGraph rosGraph;

	private ROSNetwork rosNetwork;

	private ScopeId currentNodeScopeId;

	public ROSComputationGraphDumper(
			RosComputationalGraph rosGraph, ROSNetwork n) {
		this.rosGraph = rosGraph;
		this.rosNetwork = n;
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

	public void visitActionClient(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
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
		actionName = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
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
		actionType = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
				.toString();
		actionType = actionType.substring(1,
				actionType.length() - 1);
		actionType = getMessageType(actionType, analysisState);

		ROSActionChannel channel = rosNetwork.getActionChannel(actionName);
		if (channel == null) {
			channel = new ROSActionChannel(actionName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSActionClient server = new ROSActionClient(nodeExpr.getCodeLocation().toString(), channel, actionType, expr, publisher, analysisState);
		rosNetwork.addNetworkEntity(server, nodeExpr.getCodeLocation().toString());
		var x = 3;
	}
	public void visitActionServer(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
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
		actionName = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
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
		actionType = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
				.toString();
		actionType = actionType.substring(1,
				actionType.length() - 1);
		actionType = getMessageType(actionType, analysisState);

		ROSActionChannel channel = rosNetwork.getActionChannel(actionName);
		if (channel == null) {
			channel = new ROSActionChannel(actionName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSActionServer server = new ROSActionServer(nodeExpr.getCodeLocation().toString(), channel, actionType, expr, publisher, analysisState);
		rosNetwork.addNetworkEntity(server, nodeExpr.getCodeLocation().toString());
		var x = 3;
	}
	public void visitNode(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement node, HeapExpression expr, CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool) throws Exception {
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
		nodeName = analysisState.getState()
				.getValueState()
				.eval(has, node, analysisState.getState())
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
		namespace = analysisState.getState()
				.getValueState()
				.eval(has, node, analysisState.getState())
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
			startParamService = (Boolean) analysisState.getState()
					.getValueState()
					.eval(has, node, analysisState.getState()).getConstant();
		} catch(Exception e) {}
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
			enableRosout = (Boolean) analysisState.getState()
					.getValueState()
					.eval(has, node, analysisState.getState()).getConstant();
		} catch(Exception e) {}
		ROSNode n =  new ROSNode(nodeName, namespace, startParamService, enableRosout, node, expr, analysisState, tool.getConfiguration().interproceduralAnalysis);
		rosNetwork.addEntityContainer(n);
	}

	public SymbolicExpression getNodeHeapReference(AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analyzedCFG,
	   AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState,
	   Expression node) throws SemanticException {
		String nodeName = null;
		AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> nodeSemantics = analyzedCFG.getAnalysisStateAfter(node);
		HeapReference nodeHR = new HeapReference(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE), nodeSemantics.getComputedExpressions().elements.iterator().next(), node.getLocation());
		HeapDereference nodeDeref = new HeapDereference(nodeHR.getExpression().getStaticType(), nodeHR, nodeSemantics.getComputedExpressions().elements.iterator().next().getCodeLocation());
		ExpressionSet nodeHAS = nodeSemantics
				.getState()
				.rewrite(nodeDeref,
						node,
						nodeSemantics.getState());
		return nodeHAS.iterator().next();
	}

	public void visitPublisher(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
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
		topicName = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
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
				.eval(has, publisher, analysisState.getState())
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		ROSTopic channel = rosNetwork.getTopic(topicName);
		msgType = getMessageType(msgType, analysisState);
		if (channel == null) {
			channel = new ROSTopic(topicName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSTopicPublisher p = new ROSTopicPublisher(nodeExpr.getCodeLocation().toString(), channel, msgType, publisher, expr, analysisState);
		rosNetwork.addNetworkEntity(p, nodeExpr.getCodeLocation().toString());
	}

	public String getMessageType(String variableName, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
		for (Symbol s : analysisState.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).getKeys()) {
			if (s instanceof QualifiedNameSymbol) {
				QualifiedNameSymbol qnss = (QualifiedNameSymbol) s;
				if (qnss.getName().equals(variableName)) {
					return qnss.getQualifier() + "." + qnss.getName();
				}
			}
		}
		return variableName;
	}

	public void visitServiceClient(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
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
		serviceName = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
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
		msgType = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		ROSServiceChannel channel = rosNetwork.getServiceChannel(serviceName);

		if (channel == null) {
			channel = new ROSServiceChannel(serviceName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSServiceClient s = new ROSServiceClient(nodeExpr.getCodeLocation().toString(), channel, msgType, publisher, expr, analysisState);
		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
		//ROSTopicPublisher p = new ROSTopicPublisher(nodeExpr.getCodeLocation().toString(), (ROSTopic) channel, msgType, publisher, expr, analysisState);
		//rosNetwork.addNetworkEntity(p, nodeExpr.getCodeLocation().toString());
	}

	public void visitServiceServer(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
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
		serviceName = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
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
		msgType = analysisState.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
				.toString();
		msgType = msgType.substring(1,
				msgType.length() - 1);
		msgType = getMessageType(msgType, analysisState);
		ROSServiceChannel channel = rosNetwork.getServiceChannel(serviceName);

		if (channel == null) {
			channel = new ROSServiceChannel(serviceName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSServiceServer s = new ROSServiceServer(nodeExpr.getCodeLocation().toString(), channel, msgType, publisher, expr, analysisState);
		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
		//ROSTopicPublisher p = new ROSTopicPublisher(nodeExpr.getCodeLocation().toString(), (ROSTopic) channel, msgType, publisher, expr, analysisState);
		//rosNetwork.addNetworkEntity(p, nodeExpr.getCodeLocation().toString());
	}

	public void visitSubscriber(AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, Statement publisher, HeapExpression expr, SymbolicExpression nodeExpr) throws Exception {
		String topicName = "<undefined>";
		String msgType = "<undefined>";
		//String callbackFunction = "<undefined>";
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
				.eval(has, publisher, analysisState.getState())
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
				.eval(has, publisher, analysisState.getState())
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
		String callback = analysisState
				.getState()
				.getValueState()
				.eval(has, publisher, analysisState.getState())
				.toString();
		//ROSSubscriptionCallback callbackFunction = getROSCallbackFunction(((UnresolvedCall)publisher).getSubExpressions()[4]);
		if (callback.startsWith("\"")) {
			callback = new String(callback
					.toCharArray(),
					1,
					callback.length()
							- 2);
		}
		//callbackFunction = callback;
		ROSCommunicationChannel channel = rosNetwork.getNetworkChannel(topicName);

		if (channel == null) {
			channel = new ROSTopic(topicName);
			rosNetwork.addNetworkChannel(channel);
		}
		ROSSubscriptionCallback callbackFunction = new ROSSubscriptionCallback(publisher.getCFG(), (SourceCodeLocation) publisher.getLocation(), ((UnresolvedCall)publisher).getSubExpressions()[4]);
		ROSTopicSubscription s = new ROSTopicSubscription(nodeExpr.getCodeLocation().toString(), (ROSTopic) channel, msgType, callbackFunction, publisher, expr, analysisState);
		rosNetwork.addNetworkEntity(s, nodeExpr.getCodeLocation().toString());
	}

	public void visitAnalyzedCFG(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool, AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analyzedCFG, Statement node) throws Exception {
		if (node instanceof PyNewObj) {
			PyNewObj obj = (PyNewObj) node;
			Type staticType = obj.getStaticType();
			if (staticType instanceof PyClassType) {
				PyClassType pyCObjClassType = (PyClassType) staticType;
				if (pyCObjClassType.getUnit().getImmediateAncestors().contains(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE).getUnit())) {
					// we are creating a rclpy Node.
					AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
					// in the analysis state we have a rclpy Node.
					for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
						if (expr instanceof HeapReference
								/*&& expr.getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE)))*/) {
							visitNode(analysisState, node, (HeapExpression) expr, tool);
							return;
						}
					}
				}
			}
			if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONCLIENT))) {
				AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
				for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
					if (expr instanceof HeapReference
						/*&& expr.getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE)))*/) {
						SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState, ((PyNewObj) node).getSubExpressions()[0]);
						visitActionClient(analysisState, node, (HeapExpression) expr, nodeExpr);
						return;
					}
				}
			} else if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONSERVER))) {
				AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
				for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
					if (expr instanceof HeapReference
						/*&& expr.getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE)))*/) {
						SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState, ((PyNewObj)node).getSubExpressions()[0]);
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



	public void visitNativeCFG(CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
  		NativeCFG nativeCFG, AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analyzedCFG, Statement node) throws Exception {
		String nativeCFGDescriptorName = nativeCFG.getDescriptor().getName();
		String nativeCFGDescriptorUnitName = nativeCFG.getDescriptor().getUnit().getName();
		System.out.println(" ! ! ! ! ! ! " + nativeCFGDescriptorName);
		if (node instanceof PyNewObj) {
			PyNewObj obj = (PyNewObj) node;
			Type staticType = obj.getStaticType();
			if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONCLIENT))) {
			} else if (staticType.equals(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_ACTIONSERVER))) {}
		} else if ((nativeCFGDescriptorName.equals("__init__")
				&& nativeCFGDescriptorUnitName
				.equals(LibrarySpecificationProvider.RCLPY_NODE))) {
			// Node.__init__( ... )
		} else if ((nativeCFGDescriptorName.equals("create_node")
				&& nativeCFGDescriptorUnitName
				.equals(LibrarySpecificationProvider.RCLPY))) {
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);
			for (SymbolicExpression expr : analysisState.getComputedExpressions()) {
				if (expr instanceof HeapReference
					/*&& expr.getStaticType().equals(new ReferenceType(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE)))*/) {
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
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getComputedExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
						.getStaticType()
						.equals(new ReferenceType(
								PyClassType
										.lookup(LibrarySpecificationProvider.RCLPY_SUBSCRIPTION)))) {
					//AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> nodeSemantics = analyzedCFG.getAnalysisStateAfter(unresolvedCall.getSubExpressions()[0]);
					SymbolicExpression nodeExpr = getNodeHeapReference(analyzedCFG, analysisState, unresolvedCall.getSubExpressions()[0]);
					visitSubscriber(analysisState, unresolvedCall, (HeapExpression) expr, nodeExpr);
				}
			}
		} else if (nativeCFGDescriptorName
				.equals("create_publisher")
				&& nativeCFGDescriptorUnitName
				.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getComputedExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
						.getStaticType()
						.equals(new ReferenceType(
								PyClassType
										.lookup(LibrarySpecificationProvider.RCLPY_PUBLISHER)))) {
					//AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> nodeSemantics = analyzedCFG.getAnalysisStateAfter(unresolvedCall.getSubExpressions()[0]);
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState, unresolvedCall.getSubExpressions()[0]);
					visitPublisher(analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);

					//var aigSemanticsNodeName = aigNodeName.forwardSemantics(nodeSemantics, tool.getConfiguration().interproceduralAnalysis, new StatementStore<>(nodeSemantics));
					//ConstantPropagation cp = analysisState.getState().getValueState().eval((ValueExpression) nodeSemantics.getComputedExpressions().elements.iterator().next(), node, analysisState.getState());
					// compute semantics
					//AnalysisState<A> aigSemanticsNodeName = aigNodeName.forwardSemantics(analysisState, tool.getConfiguration().interproceduralAnalysis, expressions);
					//Publisher p = visitPublisher();
					var x = 10;
				}
			}
		} else if (nativeCFGDescriptorName
				.equals("create_service")
				&& nativeCFGDescriptorUnitName
				.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getComputedExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
						.getStaticType()
						.equals(new ReferenceType(
								PyClassType
										.lookup(LibrarySpecificationProvider.RCLPY_SERVICE)))) {
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState, unresolvedCall.getSubExpressions()[0]);
					visitServiceServer(analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);
					var x = 3;
				}
			}
		} else if (nativeCFGDescriptorName.equals("create_client")
			&& nativeCFGDescriptorUnitName
			.equals(LibrarySpecificationProvider.RCLPY_NODE)) {
			UnresolvedCall unresolvedCall = (UnresolvedCall) node;
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState = analyzedCFG.getAnalysisStateAfter(node);

			for (SymbolicExpression expr : analysisState.getComputedExpressions()) {

				if (expr instanceof HeapReference
						&& ((HeapReference) expr)
						.getStaticType()
						.equals(new ReferenceType(
								PyClassType
										.lookup(LibrarySpecificationProvider.RCLPY_CLIENT)))) {
					SymbolicExpression nodeSymbolic = getNodeHeapReference(analyzedCFG, analysisState, unresolvedCall.getSubExpressions()[0]);
					visitServiceClient(analysisState, unresolvedCall, (HeapExpression) expr, nodeSymbolic);
				}
			}
		} else if (nativeCFGDescriptorName.equals("publish")
				&& nativeCFGDescriptorUnitName.equals(LibrarySpecificationProvider.RCLPY_PUBLISHER)) {
			// get the Pubisher
			// generate NetworkEvent

			Expression publisherExpression = ((UnresolvedCall) node).getSubExpressions()[0];
			Expression message = ((UnresolvedCall) node).getSubExpressions()[1];
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> publisherSemantics = analyzedCFG.getAnalysisStateAfter(publisherExpression);

			HeapReference publisherHR = new HeapReference(PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE), publisherSemantics.getComputedExpressions().elements.iterator().next(), publisherExpression.getLocation());
			HeapDereference publisherDeref = new HeapDereference(publisherHR.getExpression().getStaticType(), publisherHR, publisherSemantics.getComputedExpressions().elements.iterator().next().getCodeLocation());
			ExpressionSet publisherExprSet = publisherSemantics
					.getState()
					.rewrite(publisherDeref,
							node,
							publisherSemantics.getState());
			ROSNetworkEntity ne = rosNetwork.getNetworkEntity(publisherExprSet.iterator().next().getCodeLocation().toString());
			System.out.println(ne);
			AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> messageSemantics = analyzedCFG.getAnalysisStateAfter(message);
			Object _message = null;
			for (SymbolicExpression e : messageSemantics.getComputedExpressions()) {
				ConstantPropagation cp = messageSemantics.getState().getValueState().eval((ValueExpression) e, message , messageSemantics.getState());
				_message = cp.isTop() ? "#TOP#" : cp.getConstant();
			}
			//messageSemantics.getState().getValueState().eval(message, message.getLocation());
			NetworkMessage NetworkMessage = new NetworkMessage(_message, ne.getType());
			//NetworkEvent event = ne.createNetworkEvent(message);
			rosNetwork.createNetworkEvent(NetworkMessage, ne);
		}
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Statement node) {
		Collection<AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>>> results = tool
				.getResultOf(graph);
		try {
			for (AnalyzedCFG<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> result : results) {
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
			CheckToolWithAnalysisResults<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> tool,
			CFG graph,
			Edge edge) {
		if (graph.getDescriptor().getName().equals("$main")) {
			var nodeAnalysisState = tool.getResultOf(graph);
			// get the first result.
			var analyzedCFG = nodeAnalysisState.stream().iterator().next();

			var x = 3;
		}
		return true;
	}

	public RosComputationalGraph getRosGraph() {
		return rosGraph;
	}

	public ROSNetwork getNetwork() { return rosNetwork; }
}
