package it.unive.pylisa.libraries.fastapi.analysis.syntax;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.NullLiteral;
import it.unive.pylisa.annotationvalues.DecoratedAnnotation;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.definitions.Method;
import it.unive.pylisa.libraries.fastapi.definitions.Param;
import lombok.experimental.UtilityClass;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

@UtilityClass
public class EndpointChecker {

    public void doCheck(CheckTool tool, Unit unit, List<Endpoint> endpoints) {

        for (Endpoint endpoint : endpoints) {

            if (endpoint.getMethod() == Method.GET) validateGET(tool, unit, endpoint);
            if (endpoint.getMethod() == Method.POST) validatePOST(tool, unit, endpoint);
            if (endpoint.getMethod() == Method.PUT) validatePUT(tool, unit, endpoint);
            validateGENERL(tool, unit, endpoint);
        }
    }

    public void validateGENERL(CheckTool tool, Unit unit, Endpoint endpoint) {

        if (endpoint.getFullPath() == null) {

            String pathlessWarning = "No path for endpoint defined.";

            tool.warnOn(unit, pathlessWarning);
            endpoint.addIssue(pathlessWarning);
        }
    }

    public void validateGET(CheckTool tool, Unit unit, Endpoint endpoint) {

        String pathVariableName = endpoint.getPathVariableName();

        String methodVariableName = !endpoint.getMethodPathVariable().isEmpty()
                ? endpoint.getMethodPathVariable().get(0).getName()
                : null;

        if (pathVariableName != null && methodVariableName != null && !pathVariableName.equals(methodVariableName)) {
            tool.warnOn(unit, String.format("Path variable name of endpoint '%s' differs from one specified in method. | Path variable name: %s | Method variable name: %s", endpoint.getFullPath(), pathVariableName, methodVariableName));
            endpoint.addIssue("Path variable name differs from one specified in method.");
        }

        if (endpoint.getMethodPathVariable().isEmpty()) return;

        String methodVariableType = endpoint.getMethodPathVariable().get(0).getType();

        if (!"string".equals(methodVariableType) && !"numeric".equals(methodVariableType)) {
            tool.warnOn(unit, String.format("This GET endpoint's path method accepts non-string or non-numeric parameter: %s | Variable's type: %s", endpoint.getFullPath(), methodVariableType));
            endpoint.addIssue("Endpoint's path method accepts non-string or non-numeric parameter.");
        }
    }

    public void validatePOST(CheckTool tool, Unit unit, Endpoint endpoint) {

        if (!endpoint.getMethodPathVariable().isEmpty()) {

            boolean oneOfMethodVarsIsCustomDef = false;

            for (Param endpointVariable : endpoint.getMethodPathVariable()) {
                oneOfMethodVarsIsCustomDef = endpointVariable.isTypeCustomDefinition();
            }

            if (!oneOfMethodVarsIsCustomDef) {
                tool.warnOn(unit,"This POST endpoint " + endpoint.getFullPath() + " does not accept any argument of custom definition, t.i. model. Only primitives.");
                endpoint.addIssue("Endpoint does not accept any argument of custom definition, t.i. model. Only primitives.");
            }
        }
    }

    public void validatePUT(CheckTool tool, Unit unit, Endpoint endpoint) {

        boolean idForUpdateSpecified = false;
        boolean objectProvided = false;

        for (Param endpointVariable : endpoint.getMethodPathVariable()) {
            idForUpdateSpecified = endpointVariable.isTypeCustomDefinition();
            objectProvided = !endpointVariable.isTypeCustomDefinition();
        }

        if (!idForUpdateSpecified && objectProvided) {
            tool.warnOn(unit,"This PUT endpoint " + endpoint.getFullPath() + " does not accept any arguments to specify the object to update.");
            endpoint.addIssue("Endpoint does not accept any arguments to specify the object to update.");

        } else if (idForUpdateSpecified && !objectProvided) {
            tool.warnOn(unit,"This PUT endpoint " + endpoint.getFullPath() + " does not accept any arguments as updates target ID.");
            endpoint.addIssue("Endpoint does not accept any arguments as updates target ID.");
        }
    }

    public Boolean validateDELETE(CheckTool tool, CFG graph, Edge edge, List<Endpoint> endpoints) {

        boolean entityObtainedFromPathVar;
        boolean entityPresenceChecked;

        AnnotationMember delete = EndpointService.extractDecoratorByMethod(graph, Method.DELETE);

        if (delete != null) {

            DecoratedAnnotation decorator = (DecoratedAnnotation) delete.getValue();

            PyAssign pathWrapper = (PyAssign) decorator.getParams().get(1);
            PyStringLiteral pathLiteral = (PyStringLiteral) Arrays.stream(pathWrapper.getSubExpressions()).toList().get(1);
            String pathVariable = Endpoint.extractPathVariable(pathLiteral.getValue());

            Optional<Endpoint> endpoint = endpoints.stream().filter(e -> pathLiteral.getValue().equals(e.getFullPath())).findFirst();

            if (edge.getSource() instanceof PyIs isCheck) {

                entityObtainedFromPathVar = Arrays.stream(isCheck.getSubExpressions()).toList().get(1) instanceof NullLiteral;

                if (entityObtainedFromPathVar) {
                    VariableRef checkByNull = (VariableRef) Arrays.stream(isCheck.getSubExpressions()).toList().get(0);

                    String checkedEntityOriginId = checkByNull.getCFG().getDescriptor().getVariables().get(0).getName();
                    entityPresenceChecked = checkedEntityOriginId.equals(pathVariable);

                    if (!entityPresenceChecked) {
                        tool.warnOn(checkByNull.getCFG(), "DELETE endpoint" + pathLiteral.getValue() + " is missing prior entity check-up for its existence before deletion.");
                        endpoint.ifPresent(value -> value.addIssue("Endpoint is missing prior entity check-up for its existence before deletion."));
                    }
                }

            } else {
                tool.warnOn(edge.getSource(), "DELETE endpoint " + pathLiteral.getValue() + " deletes entity without prior check-up for its existence.");
                endpoint.ifPresent(value -> value.addIssue("Endpoint deletes entity without prior check-up for its existence."));

                if (endpoint.isPresent()) {
                    for (int i = 0; i < endpoints.size(); i++) {
                        if (endpoints.get(i).getFullPath().equals(pathLiteral.getValue())) {
                            endpoints.set(i, endpoint.get());
                            break;
                        }
                    }
                }
                return false;
            }
        }
        return true;
    }


    public void doAfterChecks(CheckTool tool, List<Endpoint> endpoints) {

        long countGET = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.GET)).count();
        long countPOST = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.POST)).count();
        long countPUT = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.PUT)).count();
        long countDELETE = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.DELETE)).count();

        tool.warn("Analysed file/s have | " +
                "GET: " + countGET + " | " +
                "POST: " + countPOST + " | " +
                "PUT: " + countPUT + " | " +
                "DELETE: " + countDELETE + " endpoints in total.");
    }
}
