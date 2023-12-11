package it.unive.pylisa.checks.semantics;

import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
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
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.pylisa.analysis.constants.ConstantPropagation;

public class RosTopicDeclarationFinder implements
		SemanticCheck<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
						TypeEnvironment<InferredTypes>>,
				PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> {

	@Override
	public void beforeExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool) {
	}

	@Override
	public void afterExecution(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool) {
	}

	@Override
	public boolean visitUnit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool,
			Unit unit) {
		return true;
	}

	@Override
	public void visitGlobal(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool,
			Unit unit, Global global, boolean instance) {

	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool,
			CFG graph) {
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool,
			CFG graph, Statement node) {
		if (node instanceof Call) {
			Call call = (Call) node;
		}
		/*
		 * if (Objects.equals(call.getTargetName(), "create_subscription") ||
		 * Objects.equals(call.getTargetName(), "create_publisher")) { String
		 * nodeType = Objects.equals(call.getTargetName(),
		 * "create_subscription") ? "Subscriber" : "Publisher"; String
		 * messageType = call.getSubExpressions()[1].toString(); String
		 * topicName = "[undefined]"; for (CFGWithAnalysisResults<
		 * SimpleAbstractState<PointBasedHeap,
		 * ValueEnvironment<ConstantPropagation>,
		 * TypeEnvironment<InferredTypes>>, PointBasedHeap,
		 * ValueEnvironment<ConstantPropagation>,
		 * TypeEnvironment<InferredTypes>> result :
		 * tool.getResultOf(call.getCFG())) { ConstantPropagation cp =
		 * result.getAnalysisStateAfter(call.getParameters()[2]) .getState()
		 * .getValueState() .getValueOnStack(); if (cp.getConstant() != null) {
		 * topicName = cp.getConstant().toString(); } } tool.warnOn(graph,
		 * nodeType +" found: " + node + ". Read from topic: " + topicName +
		 * ", message type: " + messageType); } }
		 */
		return true;
	}

	@Override
	public boolean visit(
			CheckToolWithAnalysisResults<
					SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>,
							TypeEnvironment<InferredTypes>>,
					PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>> tool,
			CFG graph, Edge edge) {
		return true;
	}
}
