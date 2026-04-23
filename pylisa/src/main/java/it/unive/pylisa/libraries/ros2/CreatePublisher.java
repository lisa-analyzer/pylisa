package it.unive.pylisa.libraries.ros2;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkResource;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.ROS2TopicCreation;
import java.util.HashMap;

/**
 * Pluggable statement for
 * {@code Node.create_publisher(self, msg_type, topic, qos_profile, ...)}.
 * Registers a publisher endpoint on the ROS2 node.
 */
public class CreatePublisher extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public CreatePublisher(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "Node.create_publisher", params, new HashMap<>() {
			{
				put("self", 0);
				put("msg_type", 1);
				put("topic", 2);
				put("qos_profile", 3);
				put("callback_group", 4);
				put("event_callbacks", 5);
				put("qos_overriding_options", 6);
				put("publisher_class", 7);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		CreatePublisher other = (CreatePublisher) o;
		int cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdVariadicSemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression[] combination,
			StatementStore<A> expressions)
			throws SemanticException {
		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ROS2TopicCreation.PUBLISHER)
				.varargsOperand("target", combination[0])
				.varargsOperand("topic", combination[getVarArgsIndex().get("topic")])
				.staticType(LiSANetworkResource.INSTANCE)
				.location(getLocation())
				.build();
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}

	public static CreatePublisher build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreatePublisher(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
