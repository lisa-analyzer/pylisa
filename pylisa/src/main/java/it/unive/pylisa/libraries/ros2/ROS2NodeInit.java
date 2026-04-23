package it.unive.pylisa.libraries.ros2;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkActiveNode;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.ROS2NodeCreation;
import it.unive.lisa.symbolic.value.Constant;
import java.util.HashMap;

/**
 * Pluggable statement for {@code Node.__init__(self, node_name, ...)}.
 * Registers the ROS2 node in the network abstract domain by emitting a
 * {@link ROS2NodeCreation} expression assigned to {@code self}
 * (combination[0]).
 */
public class ROS2NodeInit extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public ROS2NodeInit(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "Node.__init__", params, new HashMap<>() {
			{
				put("self", 0);
				put("node_name", 1);
				put("namespace", 2);
				put("context", 3);
				put("cli_args", 4);
				put("use_global_arguments", 5);
				put("enable_rosout", 6);
				put("start_parameter_services", 7);
				put("parameter_overrides", 8);
				put("allow_undeclared_parameters", 9);
				put("automatically_declare_parameters_from_overrides", 10);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		ROS2NodeInit other = (ROS2NodeInit) o;
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
		int nodeNameIdx = getVarArgsIndex().get("node_name");
		SymbolicExpression nodeName = combination.length > nodeNameIdx
				? combination[nodeNameIdx]
				: new Constant(StringType.INSTANCE, "node", getLocation());
		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(ROS2NodeCreation.INSTANCE)
				.varargsOperand("nodeName", nodeName)
				.staticType(LiSANetworkActiveNode.INSTANCE)
				.location(getLocation())
				.build();
		return interprocedural.getAnalysis().assign(state, combination[0], expr, this);
	}

	public static ROS2NodeInit build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ROS2NodeInit(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
