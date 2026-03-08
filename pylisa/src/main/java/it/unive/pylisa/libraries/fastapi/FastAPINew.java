package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariadicExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.HashMap;

public class FastAPINew extends VariadicExpression implements PluggableStatement {
	protected Statement st;

	public FastAPINew(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "FastAPINew", params, new HashMap<>() {
			{
				put("debug", 1);
				put("routes", 2);
				put("title", 3);
				put("summary", 4);
				put("description", 5);
				put("version", 6);
				put("openapi_url", 7);
				put("openapi_tags", 8);
				put("servers", 9);
				put("dependencies", 10);
				put("default_response_class", 11);
				put("redirect_slashes", 12);
				put("docs_url", 13);
				put("redoc_url", 14);
				put("swagger_ui_oauth2_redirect_url", 15);
				put("swagger_ui_init_oauth", 16);
				put("middleware", 17);
				put("exception_handlers", 18);
				put("on_startup", 19);
				put("on_shutdown", 20);
				put("lifespan", 21);
				put("terms_of_service", 22);
				put("contact", 23);
				put("license_info", 24);
				put("openapi_prefix", 25);
				put("root_path", 26);
				put("root_path_in_servers", 27);
				put("responses", 28);
				put("callbacks", 29);
				put("webhooks", 30);
				put("deprecated", 31);
				put("include_in_schema", 32);
				put("swagger_ui_parameters", 33);
				put("generate_unique_id_function", 34);
				put("separate_input_output_schemas", 35);
				put("openapi_external_docs", 36);
				put("extra", 37);
			}
		});
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		FastAPINew other = (FastAPINew) o;
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
		PyNewObj newObj = new PyNewObj(this.getCFG(), getLocation(), "__init__", PyClassType.lookup("fastapi.FastAPI"),
				getSubExpressions());
		return newObj.forwardSemantics(state, interprocedural, expressions);
		// return interprocedural.getAnalysis().assign(state, combination[0],
		// expr, this);
	}

	public static FastAPINew build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new FastAPINew(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}