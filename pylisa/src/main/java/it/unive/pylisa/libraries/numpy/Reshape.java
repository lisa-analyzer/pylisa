package it.unive.pylisa.libraries.numpy;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class Reshape extends NaryExpression implements PluggableStatement {

	protected Statement st;

	public Reshape(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "reshape", params);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		PushAny push = new PushAny(PyClassType.lookup(LibrarySpecificationProvider.NUMPY_ARRAY).getReference(),
				getLocation());
		return interprocedural.getAnalysis().smallStepSemantics(state, push, st);
	}

	public static Reshape build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Reshape(cfg, location, exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

}