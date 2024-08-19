package it.unive.pylisa.libraries.fastapi.sarl;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.libraries.fastapi.helpers.SemanticsHelpers;

public class FastAPI extends NaryExpression implements PluggableStatement {

	private Statement st;

	public FastAPI(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "FastAPI", parameters);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public Program getProgram() {
		return super.getProgram();
	}

	public static FastAPI build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new FastAPI(cfg, location, parameters);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interproceduralAnalysis,
			AnalysisState<A> analysisState,
			ExpressionSet[] expressionSets,
			StatementStore<A> statementStore)
			throws SemanticException {
		AnalysisState<A> result = analysisState.bottom();
		CodeLocation location = getLocation();
		CFG cfg = st.getCFG();
		AccessInstanceGlobal aig;
		PyAssign ass;
		Expression self = getSubExpressions()[0];
		Expression debug = SemanticsHelpers.getNamedParameterExpr(getSubExpressions(), "debug");
		if (debug == null) {
			debug = new FalseLiteral(this.getCFG(), getLocation());
		}
		Expression redirectSlashes = SemanticsHelpers.getNamedParameterExpr(getSubExpressions(), "redirect_slashes");
		if (redirectSlashes == null) {
			redirectSlashes = new TrueLiteral(this.getCFG(), getLocation());
		}
		// self.debug = true | false
		aig = new AccessInstanceGlobal(cfg, location, self, "debug");
		ass = new PyAssign(cfg, location, aig, debug);
		result = result.lub(ass.forwardSemantics(analysisState, interproceduralAnalysis, statementStore));

		// self.redirect_slashes = true | false
		aig = new AccessInstanceGlobal(cfg, location, self, "redirect_slashes");
		ass = new PyAssign(cfg, location, aig, redirectSlashes);
		result = result.lub(ass.forwardSemantics(analysisState, interproceduralAnalysis, statementStore));

		return result;
	}
}