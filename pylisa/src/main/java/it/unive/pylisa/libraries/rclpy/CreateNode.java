package it.unive.pylisa.libraries.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class CreateNode extends it.unive.lisa.program.cfg.statement.NaryExpression
		implements
		PluggableStatement {
	Statement st;

	public CreateNode(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		super(cfg, location, "create_node", exprs);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static CreateNode build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CreateNode(cfg, location, exprs);
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

		PyClassType nodeClassType = PyClassType.lookup(LibrarySpecificationProvider.RCLPY_NODE);
		PyNewObj nodeObj = new PyNewObj(this.getCFG(), (SourceCodeLocation) getLocation(),
				nodeClassType, getSubExpressions());

		AnalysisState<A> newNodeAs = nodeObj.forwardSemanticsAux(interprocedural,
				state, params, expressions);
		return newNodeAs;
	}
}
