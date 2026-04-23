package it.unive.pylisa.libraries.boto3;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.statement.ClassLiteral;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.HashMap;

/**
 * Pluggable statement for {@code boto3.client(service_name, ...)}. Acts as an
 * allocator: internally creates a {@link FunctionApply} for
 * {@code S3Client(service_name)}, which dispatches to
 * {@link ClassInstantiation} (allocates a new S3Client, then calls
 * {@code S3Client.__init__}).
 */
public class Boto3ClientFactory extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public Boto3ClientFactory(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "boto3.client", params, new HashMap<>() {
			{
				put("service_name", 1);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		Boto3ClientFactory other = (Boto3ClientFactory) o;
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
		PyClassType s3ClientType = PyClassType.lookup("boto3.S3Client");
		ClassLiteral classLit = new ClassLiteral(getCFG(), getLocation(), s3ClientType.getUnit());
		Expression serviceNameExpr = getSubExpressions()[getVarArgsIndex().get("service_name")];
		FunctionApply fa = new FunctionApply(getCFG(), getLocation(), classLit,
				new Expression[] { serviceNameExpr });
		return fa.forwardSemantics(state, interprocedural, expressions);
	}

	public static Boto3ClientFactory build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Boto3ClientFactory(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}
