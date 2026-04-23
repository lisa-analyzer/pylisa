package it.unive.pylisa.libraries.boto3;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkActiveNode;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.S3ClientCreation;
import it.unive.lisa.symbolic.value.Constant;
import java.util.HashMap;

/**
 * Pluggable statement for {@code S3Client.__init__(self, service_name, ...)}.
 * Registers the S3 client in the network abstract domain by emitting an
 * {@link S3ClientCreation} expression assigned to {@code self}
 * (combination[0]). Mirrors how
 * {@link it.unive.pylisa.libraries.fastapi.FastAPI} handles
 * {@code FastAPI.__init__}.
 */
public class S3ClientInit extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public S3ClientInit(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "S3Client.__init__", params, new HashMap<>() {
			{
				put("self", 0);
				put("service_name", 1);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		S3ClientInit other = (S3ClientInit) o;
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
		int serviceIdx = getVarArgsIndex().get("service_name");
		SymbolicExpression service = combination.length > serviceIdx
				? combination[serviceIdx]
				: new Constant(StringType.INSTANCE, "s3", getLocation());
		it.unive.lisa.symbolic.value.VariadicExpression expr = new it.unive.lisa.symbolic.value.VariadicExpression.Builder()
				.operator(S3ClientCreation.INSTANCE)
				.varargsOperand("service", service)
				.staticType(LiSANetworkActiveNode.INSTANCE)
				.location(getLocation())
				.build();
		return interprocedural.getAnalysis().assign(state, combination[0], expr, this);
	}

	public static S3ClientInit build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new S3ClientInit(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
