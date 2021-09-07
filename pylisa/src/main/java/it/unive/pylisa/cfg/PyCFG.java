package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.statement.Statement;

public class PyCFG extends CFG {
    public PyCFG(CFGDescriptor descriptor) {
        super(descriptor);
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
