package it.unive.pylisa.libraries.stdlib.object;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.cfg.expression.literal.PyNotImplementedLiteral;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ObjectIRShift extends BinaryExpression implements PluggableStatement {
	protected Statement st;

	public ObjectIRShift(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg,
				location,
				"__irshift__",
				PyClassType.lookup(LibrarySpecificationProvider.OBJECT),
				left,
				right);
	}

	public static ObjectIRShift build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectIRShift(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		return new PyNotImplementedLiteral(getCFG(), getLocation()).forwardSemantics(state, interprocedural,
				expressions);
	}
}
