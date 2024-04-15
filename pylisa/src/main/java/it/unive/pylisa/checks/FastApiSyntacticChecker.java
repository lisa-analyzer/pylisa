package it.unive.pylisa.checks;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.libraries.fastapi.models.Endpoint;
import it.unive.pylisa.libraries.fastapi.EndpointChecker;
import it.unive.pylisa.libraries.fastapi.EndpointService;

import java.util.ArrayList;

import java.util.List;

public class FastApiSyntacticChecker implements SyntacticCheck {

    public List<Endpoint> endpoints;

    @Override
    public void beforeExecution(CheckTool tool) {
        endpoints = new ArrayList<>();
    }

    @Override
    public void afterExecution(CheckTool tool) {

        EndpointChecker.doPostChecks(tool,endpoints);
    }

    @Override
    public boolean visitUnit(CheckTool tool, Unit unit) {

        if (unit instanceof CodeUnit && !unit.getName().equals("fastapi")) {

            endpoints = EndpointService.gatherEndpoints(tool, unit);


            if (!endpoints.isEmpty()) {
                EndpointChecker.doCheck(tool, unit, endpoints);
            }

            return true;
        }

        return false;
    }

    @Override
    public void visitGlobal(CheckTool tool, Unit unit, Global global, boolean instance) { }

    @Override
    public boolean visit(CheckTool tool, CFG graph) {
        
        // Check-ups for Microservice B endpoints are temporally withheld as that part was extra messy. Doing its improvements.
        return true;
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Statement node) {

        return true;
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Edge edge) {

        boolean doAnalysisContinue = EndpointChecker.validateDELETE(tool, graph, edge); // Temporary, will move no visit(Statement node) with this.
        return doAnalysisContinue;
    }
}
