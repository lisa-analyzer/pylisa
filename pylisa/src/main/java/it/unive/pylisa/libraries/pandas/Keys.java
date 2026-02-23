package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.AccessKeys;

public class Keys extends it.unive.lisa.program.cfg.statement.UnaryExpression implements PluggableStatement {

    private Statement st;

    public Keys(
            CFG cfg,
            CodeLocation location,
            Expression dataframe) {
        super(cfg, location, "keys", PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
                dataframe);
    }

    public static Keys build(
            CFG cfg,
            CodeLocation location,
            Expression[] exprs) {
        return new Keys(cfg, location, exprs[0]);
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
    //FIX ME
    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression expr, StatementStore<A> expressions) throws SemanticException {
        /*CodeLocation loc = getLocation();
        PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
        PyClassType seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
        Type seriesref = ((PyClassType) seriestype).getReference();

        HeapDereference deref = new HeapDereference(dftype, arg, loc);
        AnalysisState<A> copied = PandasSemantics.copyDataframe(state, deref, st);

        AnalysisState<A> result = state.bottom();
        for (SymbolicExpression id : copied.getComputedExpressions()) {
            UnaryExpression transform = new UnaryExpression(seriestype, id, new AccessKeys(0), loc);
            AnalysisState<A> tmp = copied.smallStepSemantics(transform, st);

            HeapReference ref = new HeapReference(seriesref, id, loc);
            result = result.lub(tmp.smallStepSemantics(ref, st));
        }
        return result;*/
        return state;
    }
}
