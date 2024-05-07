package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.lattices.Satisfiability;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;

import java.util.Objects;
import java.util.function.Predicate;

public class Imports implements SemanticDomain<Imports, SymbolicExpression, Identifier>, Lattice<Imports> {

    private static final Imports TOP = new Imports(true);
    private static final Imports BOTTOM = new Imports();

    private boolean isTop = false;
    private Import imp;
    public Imports() {}

    public Imports(boolean isTop) {
        this.isTop = isTop;
    }

    public Import getImport() {
        return imp;
    }

    public StructuredRepresentation representation() {
        if (isTop())
            return Lattice.topRepresentation();
        if (isBottom())
            return Lattice.bottomRepresentation();
        return new StringRepresentation(imp);
    }

    @Override
    public Imports top() {
        return TOP;
    }

    @Override
    public Imports bottom() {
        return BOTTOM;
    }

    @Override
    public boolean isTop() {
        return isTop;
    }
    @Override
    public boolean lessOrEqual(Imports other) throws SemanticException {
        return Objects.equals(imp, other.imp);
    }

    @Override
    public Imports lub(Imports other) throws SemanticException {
        return Objects.equals(imp, other.imp) ? this : top();
    }

    @Override
    public Imports assign(Identifier id, SymbolicExpression expression, ProgramPoint pp, SemanticOracle oracle) throws SemanticException {
        return null;
    }

    @Override
    public Imports smallStepSemantics(SymbolicExpression expression, ProgramPoint pp, SemanticOracle oracle) throws SemanticException {
        return null;
    }

    @Override
    public Imports assume(SymbolicExpression expression, ProgramPoint src, ProgramPoint dest, SemanticOracle oracle) throws SemanticException {
        return null;
    }

    @Override
    public boolean knowsIdentifier(Identifier id) {
        return false;
    }

    @Override
    public Imports forgetIdentifier(Identifier id) throws SemanticException {
        return null;
    }

    @Override
    public Imports forgetIdentifiersIf(Predicate<Identifier> test) throws SemanticException {
        return null;
    }

    @Override
    public Satisfiability satisfies(SymbolicExpression expression, ProgramPoint pp, SemanticOracle oracle) throws SemanticException {
        return null;
    }

    @Override
    public Imports pushScope(ScopeToken token) throws SemanticException {
        return null;
    }

    @Override
    public Imports popScope(ScopeToken token) throws SemanticException {
        return null;
    }
}
