package it.unive.pylisa.checks;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.NullLiteral;
import it.unive.pylisa.annotationvalues.DecoratedAnnotation;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.libraries.fastapi.model.Endpoint;
import it.unive.pylisa.libraries.fastapi.model.Method;
import it.unive.pylisa.libraries.fastapi.model.Param;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.StreamSupport;

public class FastApiSyntacticChecker implements SyntacticCheck {

    private List<Endpoint> endpoints;

    @Override
    public void beforeExecution(CheckTool tool) {
        endpoints = new ArrayList<>();
    }

    @Override
    public void afterExecution(CheckTool tool) {

        long endpointCountGET = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.GET)).count();
        long endpointCountPOST = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.POST)).count();
        long endpointCountPUT = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.PUT)).count();
        long endpointCountDELETE = endpoints.stream().filter(endpoint -> endpoint.getMethod().equals(Method.DELETE)).count();

        tool.warn("Analysed file/s have | GET: " + endpointCountGET + " | POST: " + endpointCountPOST + " | PUT: "
                + endpointCountPUT + "| DELETE: " + endpointCountDELETE + " endpoints in total.");
    }

    @Override
    public boolean visitUnit(CheckTool tool, Unit unit) {

        if (unit instanceof CodeUnit && !unit.getName().equals("fastapi")) {

            int index = 0;

            for (CodeMember member : unit.getCodeMembers()) {

                CodeMemberDescriptor descriptor = member.getDescriptor();
                this.specifyRestInfoFromDecorators(index, descriptor);

                for (Parameter parameter : descriptor.getFormals()) {
                    this.specifyMethodArguments(index, (PyParameter) parameter);
                }

                if (endpoints.size() != index) {
                    this.doCheck(tool, descriptor, endpoints.get(index));
                    index++;
                }
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

        boolean entityObtainedFromPathVar = false;
        boolean entityPresenceChecked = false;

        AnnotationMember delete = extractDecoratorByRestMethod(graph, Method.DELETE);

        if (delete != null) {

            DecoratedAnnotation decorator = (DecoratedAnnotation) delete.getValue();

            PyAssign pathWrapper = (PyAssign) decorator.getParams().get(0);
            PyStringLiteral pathLiteral = (PyStringLiteral) Arrays.stream(pathWrapper.getSubExpressions()).toList().get(1);
            String pathVariable = Endpoint.extractPathVariable(pathLiteral.getValue());

            if (edge.getSource() instanceof PyIs isCheck) {

                entityObtainedFromPathVar = Arrays.stream(isCheck.getSubExpressions()).toList().get(1) instanceof NullLiteral;

                if (entityObtainedFromPathVar) {
                    VariableRef checkByNull = (VariableRef) Arrays.stream(isCheck.getSubExpressions()).toList().get(0);

                    String checkedEntityOriginId = checkByNull.getCFG().getDescriptor().getVariables().get(0).getName();
                    entityPresenceChecked = checkedEntityOriginId.equals(pathVariable);

                    if (!entityPresenceChecked) {
                        tool.warnOn(checkByNull.getCFG(), "DELETE endpoint" + pathLiteral.getValue() + " is missing prior entity check-up for its existence before deletion.");
                    }
                }
            } else {
                tool.warnOn(edge.getSource(), "DELETE endpoint " + pathLiteral.getValue() + " deletes entity without prior check-up for its existence.");
                return false;
            }
        }
        return true;
    }

    private AnnotationMember extractDecoratorByRestMethod (CFG graph, Method method) {

        Annotations annotations = graph.getDescriptor().getAnnotations();

        List<Annotation> decorators = StreamSupport.stream(annotations.spliterator(), false)
                .filter(annotation -> annotation.getAnnotationName().equals("$decorators"))
                .toList();

        for (Annotation annotation : decorators) {
            Optional<AnnotationMember> decoratorByMethod = annotation.getAnnotationMembers().stream()
                    .filter(member -> member.getId().contains(method.getValue()))
                    .findFirst();

            if (decoratorByMethod.isPresent()) {
                return decoratorByMethod.get();
            }
        }
        return null;
    }

    private void specifyRestInfoFromDecorators(int i, CodeMemberDescriptor descriptor) {

        Annotations annotations = descriptor.getAnnotations();

        List<Annotation> decorators = StreamSupport.stream(annotations.spliterator(), false)
                .filter(annotation -> annotation.getAnnotationName().equals("$decorators"))
                .toList();

        for (Annotation annotation : decorators) {

            List<String> restKeywords = List.of("get", "post", "delete", "put");

            Optional<AnnotationMember> restDecorator = annotation.getAnnotationMembers().stream()
                    .filter(member -> restKeywords.stream().anyMatch(member.getId()::contains))
                    .findFirst();

            if (restDecorator.isPresent()) {

                Endpoint endpoint = new Endpoint(restDecorator.get());
                endpoints.add(i, endpoint);
            }
        }
    }

    private void specifyMethodArguments(int i, PyParameter parameter) {

        Param param = new Param(parameter.getTypeHint(), parameter.getName());
        endpoints.get(i).getMethodPathVariable().add(param);
    }

    private void doCheck(CheckTool tool, CodeMemberDescriptor descriptor, Endpoint endpoint) {

        this.generalChecks(tool, descriptor, endpoint);
        if (endpoint.getMethod() == Method.GET) this.getChecks(tool, descriptor, endpoint);
        if (endpoint.getMethod() == Method.POST) this.postChecks(tool, descriptor, endpoint);
        if (endpoint.getMethod() == Method.PUT) this.putChecks(tool, descriptor, endpoint);
    }

    private void generalChecks(CheckTool tool, CodeMemberDescriptor descriptor, Endpoint endpoint) {

        if (endpoint.getFullPath() == null) {
            tool.warnOn(descriptor, "No path for endpoint defined");
        }
    }

    private void getChecks(CheckTool tool, CodeMemberDescriptor descriptor, Endpoint endpoint) {

        String pathVariableName = endpoint.getPathVariableName();

        String methodVariableName = !endpoint.getMethodPathVariable().isEmpty()
                ? endpoint.getMethodPathVariable().get(0).getName()
                : null;

        if (pathVariableName != null && methodVariableName != null && !pathVariableName.equals(methodVariableName)) {
            tool.warnOn(descriptor, String.format("Path variable name of endpoint '%s' differs from one specified in method. | Path variable name: %s | Method variable name: %s", endpoint.getFullPath(), pathVariableName, methodVariableName));
        }

        if (endpoint.getMethodPathVariable().isEmpty()) return;

        String methodVariableType = endpoint.getMethodPathVariable().get(0).getType();

        if (!"string".equals(methodVariableType) && !"numeric".equals(methodVariableType)) {
            tool.warnOn(descriptor, String.format("This GET endpoint's path method accepts non-string or non-numeric parameter: %s | Variable's type: %s", endpoint.getFullPath(), methodVariableType));
        }
    }

    private void postChecks(CheckTool tool, CodeMemberDescriptor descriptor, Endpoint endpoint) {

        if (!endpoint.getMethodPathVariable().isEmpty()) {

            boolean oneOfMethodVarsIsCustomDef = false;

            for (Param endpointVariable : endpoint.getMethodPathVariable()) {
                oneOfMethodVarsIsCustomDef = endpointVariable.isTypeCustomDefinition();
            }

            if (!oneOfMethodVarsIsCustomDef) {
                tool.warnOn(descriptor,"This POST endpoint " + endpoint.getFullPath() + " does not accept any argument of custom definition / model. Only primitives.");
            }
        }
    }

    private void putChecks(CheckTool tool, CodeMemberDescriptor descriptor, Endpoint endpoint) {

        boolean idForUpdateSpecified = false;
        boolean objectProvided = false;

        for (Param endpointVariable : endpoint.getMethodPathVariable()) {
            idForUpdateSpecified = endpointVariable.isTypeCustomDefinition();
            objectProvided = !endpointVariable.isTypeCustomDefinition();
        }

        if (!idForUpdateSpecified && objectProvided) {
            tool.warnOn(descriptor,"This PUT endpoint " + endpoint.getFullPath() + " does not accept any arguments to specify the object to update.");

        } else if (idForUpdateSpecified && !objectProvided) {
            tool.warnOn(descriptor,"This PUT endpoint " + endpoint.getFullPath() + " does not accept any arguments as updates target id.");
        }
    }
}
