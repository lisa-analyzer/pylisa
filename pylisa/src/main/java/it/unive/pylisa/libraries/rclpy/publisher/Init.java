package it.unive.pylisa.libraries.rclpy.publisher;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyAssign;
import java.util.Set;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
	protected Statement st;

	public Init(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		super(cfg, location, "__init__", exprs);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static Init build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new Init(cfg, location, exprs);
	}

	@Override
	public String toString() {
		return "__init__";
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();

		for (SymbolicExpression v : params[0]) {
			Set<Type> rts = state.getState().getRuntimeTypesOf(v, this, state.getState());
			for (Type recType : rts)
				if (recType.isPointerType()) {
					Type inner = recType.asPointerType().getInnerType();
					if (!inner.isUnitType())
						continue;

					AnalysisState<A> partial = state;

					HeapDereference container = new HeapDereference(inner, v, getLocation());
					CompilationUnit unit = inner.asUnitType().getUnit();

					Global global = new Global(getLocation(), unit, "msg_type", false, StringType.INSTANCE);
					Variable var = global.toSymbolicVariable(getLocation());
					AccessChild access = new AccessChild(var.getStaticType(), container, var, getLocation());
					AnalysisState<A> tmp = state.bottom();
					SymbolicExpression msgType = params[1].iterator().next();
					for (SymbolicExpression t : params[1])
						tmp = tmp.lub(partial.assign(access, t, this));
					partial = tmp;

					global = new Global(getLocation(), unit, "topic_name", false, StringType.INSTANCE);
					var = global.toSymbolicVariable(getLocation());
					access = new AccessChild(var.getStaticType(), container, var, getLocation());
					tmp = state.bottom();
					for (SymbolicExpression t : params[2])
						tmp = tmp.lub(partial.assign(access, t, this));
					partial = tmp;

					global = new Global(getLocation(), unit, "qos_profile", false, Untyped.INSTANCE);
					var = global.toSymbolicVariable(getLocation());
					access = new AccessChild(var.getStaticType(), container, var, getLocation());
					tmp = state.bottom();
					for (SymbolicExpression t : params[3])
						tmp = tmp.lub(partial.assign(access, t, this));
					partial = tmp;

					tmp = state.bottom();
					for (SymbolicExpression t : params[3])
						tmp = tmp.lub(partial.assign(access, t, this));
					partial = tmp;
					global = new Global(getLocation(), unit, "_node", false, Untyped.INSTANCE);
					var = global.toSymbolicVariable(getLocation());
					access = new AccessChild(var.getStaticType(), container, var, getLocation());

					result = result.lub(partial);
				}
		}

		return result;
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

}