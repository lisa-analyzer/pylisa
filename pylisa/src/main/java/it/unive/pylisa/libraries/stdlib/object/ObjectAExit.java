package it.unive.pylisa.libraries.stdlib.object;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.type.VoidType;

public class ObjectAExit extends NaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectAExit(
			CFG cfg,
			CodeLocation location,
			Expression self,
			Expression exc_type,
			Expression exc_value,
			Expression traceback) {
		super(cfg, location, "__aexit__", VoidType.INSTANCE, self, exc_type, exc_value, traceback);
	}

	public static ObjectAExit build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectAExit(cfg, location, exprs[0], exprs[1], exprs[2], exprs[3]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// nothing we can do here really
		return state;
	}
}
