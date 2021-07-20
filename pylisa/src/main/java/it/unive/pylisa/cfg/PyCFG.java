package it.unive.pylisa.cfg;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.AdjacencyMatrix;
import it.unive.lisa.util.datastructures.graph.Node;

import java.util.Collection;

public class PyCFG extends CFG {
    public PyCFG(CFGDescriptor descriptor) {
        super(descriptor);
    }

    public PyCFG(CFGDescriptor descriptor, Collection<Statement> entrypoints, AdjacencyMatrix<Statement, Edge, CFG> adjacencyMatrix) {
        super(descriptor, entrypoints, adjacencyMatrix);
    }

    protected PyCFG(CFG other) {
        super(other);
    }

    public void addNodeIfNotPresent(Statement n) {
        if(! this.adjacencyMatrix.getNodes().contains(n))
            this.addNode(n);
    }
    public void addNodeIfNotPresent(Statement n, boolean entrypoint) {
        this.addNodeIfNotPresent(n);
        if(entrypoint)
            this.entrypoints.add(n);
    }
}
