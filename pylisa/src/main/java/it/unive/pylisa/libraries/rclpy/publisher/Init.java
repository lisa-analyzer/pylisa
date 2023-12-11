package it.unive.pylisa.libraries.rclpy.publisher;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyAssign;
import java.util.Collections;

public class Init extends it.unive.lisa.program.cfg.statement.NaryExpression implements PluggableStatement {
	protected Statement st;

	public Init(CFG cfg, CodeLocation location, Expression[] exprs) {
		super(cfg, location, "__init__", exprs);
	}

	public static Init build(CFG cfg, CodeLocation location, Expression[] exprs) {
		return new Init(cfg, location, exprs);
	}

	@Override
	public String toString() {
		return "__init__";
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural, AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params, StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = state.bottom();
		AccessInstanceGlobal aig;
		PyAssign pa;

		for (SymbolicExpression v : params[0]) {
			for (Type recType : v.getRuntimeTypes(getCFG().getDescriptor().getUnit().getProgram().getTypes()))
				if (recType.isPointerType()) {
					Type inner = recType.asPointerType().getInnerType();
					if (!inner.isUnitType())
						continue;

					AnalysisState<A, H, V, T> partial = state;

					HeapDereference container = new HeapDereference(inner, v, getLocation());
					container.setRuntimeTypes(Collections.singleton(inner));
					CompilationUnit unit = inner.asUnitType().getUnit();

					Global global = new Global(getLocation(), unit, "msg_type", false, StringType.INSTANCE);
					Variable var = global.toSymbolicVariable(getLocation());
					AccessChild access = new AccessChild(var.getStaticType(), container, var, getLocation());
					AnalysisState<A, H, V, T> tmp = state.bottom();
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

					global = new Global(getLocation(), unit, "qos_profile", false, StringType.INSTANCE);
					var = global.toSymbolicVariable(getLocation());
					access = new AccessChild(var.getStaticType(), container, var, getLocation());
					tmp = state.bottom();
					for (SymbolicExpression t : params[3])
						tmp = tmp.lub(partial.assign(access, t, this));
					partial = tmp;

					result = result.lub(partial);
				}
		}

		return result;
	}

	@Override
	public void setOriginatingStatement(Statement st) {
		this.st = st;
	}

}