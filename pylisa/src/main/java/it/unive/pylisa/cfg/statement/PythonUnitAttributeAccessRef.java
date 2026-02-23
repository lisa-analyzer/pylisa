package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.program.ModuleUnit;

public class PythonUnitAttributeAccessRef extends Expression {

    /**
     * The receiver of the access
     */
    private final CompilationUnit compilationUnit;

    /**
     * The global being accessed
     */
    private final Global target;

    /**
     * Builds the global access, happening at the given location in the program.
     * The type of this expression is the one of the accessed global.
     *
     * @param cfg       the cfg that this expression belongs to
     * @param location  the location where the expression is defined within the
     *                      program
     * @param compilationUnit the unit containing the accessed global
     * @param target    the accessed global
     */
    public PythonUnitAttributeAccessRef(
            CFG cfg,
            CodeLocation location,
            CompilationUnit compilationUnit,
            Global target) {
        super(cfg, location, target.getStaticType());
        this.compilationUnit = compilationUnit;
        this.target = target;
    }

    /**
     * Yields the {@link Unit} where the global targeted by this access is
     * defined.
     *
     * @return the container of the global
     */
    public CompilationUnit getContainer() {
        return compilationUnit;
    }

    /**
     * Yields the {@link Global} targeted by this expression.
     *
     * @return the global
     */
    public Global getTarget() {
        return target;
    }

    @Override
    public <V> boolean accept(
            GraphVisitor<CFG, Statement, Edge, V> visitor,
            V tool) {
        return visitor.visit(tool, getCFG(), this);
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = super.hashCode();
        result = prime * result + ((compilationUnit == null) ? 0 : compilationUnit.hashCode());
        result = prime * result + ((target == null) ? 0 : target.hashCode());
        return result;
    }

    @Override
    public boolean equals(
            Object obj) {
        if (this == obj)
            return true;
        if (!super.equals(obj))
            return false;
        if (getClass() != obj.getClass())
            return false;
        PythonUnitAttributeAccessRef other = (PythonUnitAttributeAccessRef) obj;
        if (compilationUnit == null) {
            if (other.compilationUnit != null)
                return false;
        } else if (!compilationUnit.equals(other.compilationUnit))
            return false;
        if (target == null) {
            if (other.target != null)
                return false;
        } else if (!target.equals(other.target))
            return false;
        return true;
    }

    @Override
    protected int compareSameClass(
            Statement o) {
        PythonUnitAttributeAccessRef other = (PythonUnitAttributeAccessRef) o;
        int cmp;
        if ((cmp = compilationUnit.getName().compareTo(other.compilationUnit.getName())) != 0)
            return cmp;
        return target.getName().compareTo(other.target.getName());
    }

    @Override
    public String toString() {
        return compilationUnit.getName() + "::" + target.getName();
    }

    @Override
    public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
            AnalysisState<A> state,
            InterproceduralAnalysis<A, D> interprocedural,
            StatementStore<A> expressions)
            throws SemanticException {
        Analysis<A, D> analysis = interprocedural.getAnalysis();

        GlobalVariable access = new GlobalVariable(
                target.getStaticType(),
                "$" + compilationUnit.getName() + "::" + target.getName(),
                target.getAnnotations(),
                getLocation());
        CodeLocation loc = getLocation();
        return analysis.smallStepSemantics(state, access, this);
    }
}
