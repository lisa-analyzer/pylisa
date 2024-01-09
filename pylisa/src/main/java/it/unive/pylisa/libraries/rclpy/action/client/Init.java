package it.unive.pylisa.libraries.rclpy.action.client;

import java.util.Set;

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
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.libraries.rclpy.node.SemanticsHelpers;

public class Init extends NaryExpression implements PluggableStatement {
  protected Statement st;

  public Init(CFG cfg, CodeLocation location, Expression[] exprs) {
    super(cfg, location, "__init__", exprs);
  }

  public static Init build(CFG cfg,
      CodeLocation location,
      Expression[] exprs) {
    return new Init(cfg, location, exprs);
  }

  @Override
  public void setOriginatingStatement(Statement st) {
    this.st = st;
  }

  @Override
  protected int compareSameClassAndParams(Statement o) {
    return 0;
  }

  @Override
  public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(InterproceduralAnalysis<A> interprocedural,
      AnalysisState<A> state, ExpressionSet[] params, StatementStore<A> expressions) throws SemanticException {
    AnalysisState<A> result = state.bottom();
    Expression node = getSubExpressions()[1] instanceof NamedParameterExpression
        ? SemanticsHelpers.getNamedParameterExpr(getSubExpressions(),
            "node").getSubExpression()
        : getSubExpressions()[1]; // The 'Node'
    ExpressionSet nameExpansion = SemanticsHelpers.nameExpansion(this, node,
        params[3], interprocedural, state, expressions);
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

          Global global = new Global(getLocation(), unit, "action_type", false, StringType.INSTANCE);
          Variable var = global.toSymbolicVariable(getLocation());
          AccessChild access = new AccessChild(var.getStaticType(), container, var, getLocation());
          AnalysisState<A> tmp = state.bottom();
          for (SymbolicExpression t : params[2])
            tmp = tmp.lub(partial.assign(access, t, this));
          partial = tmp;
          global = new Global(getLocation(), unit, "action_name", false, StringType.INSTANCE);
          var = global.toSymbolicVariable(getLocation());
          access = new AccessChild(var.getStaticType(), container, var, getLocation());
          tmp = state.bottom();
          for (SymbolicExpression t : nameExpansion)
            tmp = tmp.lub(partial.assign(access, t, this));
          partial = tmp;
          result = result.lub(partial);
        }
    }
    // Expression self = getSubExpressions()[0]; // The 'ActionClient'
    // Expression node = getSubExpressions()[1] instanceof NamedParameterExpression
    // ? SemanticsHelpers.getNamedParameterExpr(getSubExpressions(),
    // "node").getSubExpression()
    // : getSubExpressions()[1]; // The 'Node'
    // Expression actionNameExp = getSubExpressions()[3] instanceof
    // NamedParameterExpression
    // ? SemanticsHelpers.getNamedParameterExpr(getSubExpressions(),
    // "action_name").getSubExpression()
    // : getSubExpressions()[3]; // The 'Action name'
    // ExpressionSet nameExpansion = SemanticsHelpers.nameExpansion(this, node,
    // params[3], interprocedural, state,
    // expressions); // Expand the name of the action with node name and namespace
    // AccessInstanceGlobal actionName = new AccessInstanceGlobal(st.getCFG(),
    // getLocation(), self, "action_name");
    // AnalysisState<A> tmp = result;
    // for (SymbolicExpression se : nameExpansion) {
    // tmp.lub(tmp.assign(se, actionName, actionNameExp));
    // }
    // tmp = tmp.lub(partial.assign(access, t, this));
    // partial = tmp;

    // PyAssign pyAssign = new PyAssign(getCFG(), getLocation(), self,
    // actionNameExp);
    // result = result.lub(pyAssign.forwardSemantics(state, interprocedural,
    // expressions));

    return result;
    // return result;
  }

}
