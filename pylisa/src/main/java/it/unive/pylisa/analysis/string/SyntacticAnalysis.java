package it.unive.pylisa.analysis.string;


import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;

public class SyntacticAnalysis implements SyntacticCheck {
    @Override
    public void beforeExecution(CheckTool tool) {

    }

    @Override
    public void afterExecution(CheckTool tool) {

    }

    @Override
    public boolean visitCompilationUnit(CheckTool tool, CompilationUnit unit) {
        return true;
    }

    @Override
    public void visitGlobal(CheckTool tool, Unit unit, Global global, boolean instance) {

    }

    @Override
    public boolean visit(CheckTool tool, CFG graph) {
        return true;
    }


    @Override
    public boolean visit(CheckTool tool, CFG graph, Statement node) {
        if(node instanceof Call) {
            Call c = (Call) node;
            if(c.getTargetName().equals("create_publisher"))
                tool.warnOn(node, "Creating publisher on topic "+extractTopic(c.getParameters()[2]));
            else if(c.getTargetName().equals("create_subscription"))
                tool.warnOn(node, "Creating subscriber on topic "+extractTopic(c.getParameters()[2]));
            else if(c.getTargetName().equals("__init__"))
                tool.warnOn(node, "Creating node "+extractTopic(c.getParameters()[1]));
        }
        return true;
    }

    private String extractTopic(Expression parameter) {
        if(parameter instanceof StringLiteral)
            return ((StringLiteral) parameter).getValue();
        else return "<unknown>";
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Edge edge) {
        return true;
    }
}
