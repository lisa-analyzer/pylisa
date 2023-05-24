package it.unive.pylisa.checks;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;

public class TestFunctionFinder implements SyntacticCheck {

    private int count;
    @Override
    public void beforeExecution(CheckTool tool) {
        // initialization
        count = 0;
    }

    @Override
    public void afterExecution(CheckTool tool) {
        tool.warn("Found " + count + " functions with name test");
    }

    @Override
    public boolean visitUnit(CheckTool tool, Unit unit) {
        // A unit could be a Class. This method should return true since we could have a function test defined inside it.
        return true;
    }

    @Override
    public void visitGlobal(CheckTool tool, Unit unit, Global global, boolean instance) {
        // here we do nothing: we are not considering globals (i.e. variables)
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph) {

        if (graph.getDescriptor().getName().equals("test")) {
            count += 1;
            tool.warnOn(graph, "test function found!");
        }
        // we want to perform analysis only on the signature of the function: it is not necessary to visiting also the cfg.
        return false;
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Statement node) {
        return false;
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Edge edge) {
        return false;
    }
}
