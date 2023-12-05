package it.unive.pylisa.libraries;

import java.util.HashSet;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;

public class Super extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {
	protected Statement st;

	protected Super(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression[] expressions) {
		super(cfg, location, constructName, expressions[0], expressions[1]);
	}

	public static Super build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Super(cfg, location, "super", exprs);
	}

	@Override
	public String toString() {
		return "super";
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {

		CompilationUnit superObj = ((CompilationUnit) ((Constant) left).getValue()).getImmediateAncestors().iterator()
				.next();
		PyClassType classTypeTo = PyClassType.lookup(superObj.getName());
		ReferenceType refTypeTo = new ReferenceType(classTypeTo);
		HashSet<Type> tokenTypes = new HashSet<>();
		tokenTypes.add(refTypeTo);
		PyTypeTokenType tokenTypeTo = new PyTypeTokenType(tokenTypes);
		BinaryExpression be = new it.unive.lisa.symbolic.value.BinaryExpression(
				refTypeTo,
				right,
				new Constant(tokenTypeTo, superObj, getLocation()),
				TypeConv.INSTANCE,
				getLocation());
		return state.smallStepSemantics(be, this);
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}
}