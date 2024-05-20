package it.unive.pylisa.checks;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.libraries.fastapi.helpers.TextHelper;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.analysis.syntax.helpers.EndpointChecker;
import it.unive.pylisa.libraries.fastapi.analysis.syntax.EndpointService;

import java.util.*;

public class FastApiSyntacticChecker implements SyntacticCheck {

    public final  List<Endpoint> endpoints = new ArrayList<>();

    @Override
    public void beforeExecution(CheckTool tool) { }

    @Override
    public void afterExecution(CheckTool tool) {
        EndpointChecker.doAfterChecks(tool, endpoints);
    }

    @Override
    public boolean visitUnit(CheckTool tool, Unit unit) {

        if (unit instanceof CodeUnit && !unit.getName().equals("fastapi")) {

            List<Endpoint> unitEndpoints = EndpointService.gatherProviderEndpoints(unit);

            if (!unitEndpoints.isEmpty()) {
                endpoints.addAll(unitEndpoints);
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

        Endpoint endpoint = EndpointService.gatherConsumerEndpoint(node);

        if (endpoint != null) {
            String sourceName = TextHelper.getFilenameFromUnit(graph.getDescriptor().getUnit());

            endpoint.setBelongs(sourceName);
            endpoints.add(endpoint);
        }

        return true;
    }

    @Override
    public boolean visit(CheckTool tool, CFG graph, Edge edge) {

        boolean doAnalysisContinue = EndpointChecker.validateDELETE(tool, graph, edge, endpoints); // Temporary, will move no visit(Statement node) with this.
        return true;
    }
}
