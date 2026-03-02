package it.unive.pylisa.libraries.builtins.object;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Untyped;

public class ObjectNew extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {
	private Statement st;

	public ObjectNew(
			CFG cfg,
			CodeLocation location,
			Expression cls) {
		super(cfg, location, "__new__", Untyped.INSTANCE,
				cls);
	}

	public static ObjectNew build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectNew(cfg, location, exprs[0]);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	// FIX ME
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		System.out.println("OBJECT NEW!!!!");
		return state;
	}
}
