package it.unive.pylisa.cfg.statement;


import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.FunctionUnit;

public class ImportFunction extends Expression {
    private String className;
    private FunctionUnit functionUnit;

    /**
     * Builds a statement happening at the given source location.
     *
     * @param cfg      the cfg that this statement belongs to
     * @param cfg      the cfg that this statement belongs to
     * @param location the location where this statement is defined within the
     *                 program
     */
    protected ImportFunction(CFG cfg, CodeLocation location) {
        super(cfg, location);
    }

    public ImportFunction(CFG cfg, CodeLocation location, String className, FunctionUnit functionUnit) {
        this(cfg, location);
        this.className = className;
        this.functionUnit = functionUnit;

    }

    @Override
    public String toString() {
        return "<function> " + className;
    }

    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(AnalysisState<A> entryState, InterproceduralAnalysis<A, D> interprocedural, StatementStore<A> expressions) throws SemanticException {
        AnalysisState<A> state = entryState;
        state = ObjectRegister.initialize(state, this, functionUnit, interprocedural);
        VariableRef v = new VariableRef(this.getCFG(),  getLocation(), "$" + functionUnit.getName());
        PyAssign assign = new PyAssign(this.getCFG(), getLocation(), v, new FunctionLiteral(this.getCFG(), getLocation(), functionUnit));
        state = assign.forwardSemantics(state, interprocedural, expressions);
        return state;
    }

    @Override
    protected int compareSameClass(Statement o) {
        return 0;
    }

    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        return visitor.visit(tool, getCFG(), this);
    }

    public FunctionUnit getFunctionUnit() { return functionUnit;}
}
