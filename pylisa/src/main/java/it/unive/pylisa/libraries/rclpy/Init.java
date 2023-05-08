package it.unive.pylisa.libraries.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;


/**
 * rospy.init() (function that we don't want to model and that returns nothing).
 */
public class Init extends Expression implements PluggableStatement {
    protected Statement st;
    
    protected Init(CFG cfg, CodeLocation location) {
        super(cfg, location);
    }
    
    public static Init build(CFG cfg, CodeLocation location) {
        return new Init(cfg, location);
    }
    @Override
    public String toString() {
        return "init";
    }
    
    @Override
    public <A extends AbstractState<A, H, V, T>, H extends HeapDomain<H>, V extends ValueDomain<V>, T extends TypeDomain<T>> AnalysisState<A, H, V, T> semantics(AnalysisState<A, H, V, T> entryState, InterproceduralAnalysis<A, H, V, T> interprocedural, StatementStore<A, H, V, T> expressions) throws SemanticException {
        return entryState.smallStepSemantics(new Skip(getLocation()), this);
    }
    
    
    @Override
    public void setOriginatingStatement(Statement st) {
        this.st = st;
    }
    
    @Override
    public int setOffset(int offset) {
        this.offset = offset;
        return offset;
    }
    
    @Override
    public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
        return visitor.visit(tool, getCFG(), this);
    }
}
