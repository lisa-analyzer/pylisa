package it.unive.pylisa.libraries.boto3;

import it.unive.lisa.analysis.*;
import it.unive.lisa.cfg.type.LiSANetworkResource;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.operators.network.S3ResourceCreation;
import java.util.HashMap;

/**
 * Pluggable statement for
 * {@code s3_client.put_object(Bucket=..., Key=..., Body=...)}.
 */
public class S3PutObject extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public S3PutObject(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "s3.put_object", params, new HashMap<>() {
			{
				put("Bucket", 1);
				put("Key", 2);
				put("Body", 3);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		S3PutObject other = (S3PutObject) o;
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
				.operator(S3ResourceCreation.PUT_OBJECT)
				.varargsOperand("target", combination[0])
				.varargsOperand("bucket", combination[getVarArgsIndex().get("Bucket")])
				.varargsOperand("key", combination[getVarArgsIndex().get("Key")])
				.staticType(LiSANetworkResource.INSTANCE)
				.location(getLocation())
				.build();
		return interprocedural.getAnalysis().smallStepSemantics(state, expr, this);
	}

	public static S3PutObject build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new S3PutObject(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
