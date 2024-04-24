package it.unive.pylisa.libraries.fastapi.analysis.syntax;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.definitions.Method;
import it.unive.pylisa.libraries.fastapi.definitions.Param;
import it.unive.pylisa.libraries.fastapi.definitions.Role;
import lombok.experimental.UtilityClass;

import org.apache.http.client.utils.URIBuilder;

import java.util.*;
import java.util.stream.StreamSupport;

@UtilityClass
public class EndpointService {

    public List<Endpoint> endpoints = new ArrayList<>();
    public Set<String> providedPaths = new HashSet<>();

    public List<Endpoint> gatherProviderEndpoints(Unit unit) {

        int index = 0;

        for (CodeMember member : unit.getCodeMembers()) {

            CodeMemberDescriptor descriptor = member.getDescriptor();
            EndpointService.extractInfoFromDecorators(index, descriptor);

            for (Parameter parameter : descriptor.getFormals()) {
                specifyMethodsArguments(index, (PyParameter) parameter);
            }
        }

        return endpoints;
    }

    public Endpoint gatherConsumerEndpoint(Statement node) {

        if (node instanceof PyStringLiteral pyStringLiteral) {

            if (pyStringLiteral.getParentStatement() instanceof UnresolvedCall parent) {

                if (Arrays.stream(parent.getSubExpressions()).toList().get(0) instanceof VariableRef variableRef) {

                    if (variableRef.getName().equals("requests")) {

                        PyStringLiteral endpointInfo = (PyStringLiteral) Arrays.stream(parent.getSubExpressions()).toList().get(1);

                        URIBuilder url = null;

                        try {
                            url = new URIBuilder(endpointInfo.getValue());
                        } catch (Exception ignored) {}

                        Endpoint endpoint = new Endpoint();
                        endpoint.setMethod(Method.specify(parent.getConstructName()));
                        endpoint.setRole(Role.CONSUMER);
                        endpoint.setFullPath(url.getPath());

                        return endpoint;
                    }
                }
            }
        }

        return null;
    }

    private void specifyMethodsArguments(int i, PyParameter parameter) {

        Param param = new Param(parameter.getTypeHint(), parameter.getName());
        endpoints.get(i).getMethodPathVariable().add(param);
    }

    public AnnotationMember extractDecoratorByMethod(CFG graph, Method method) {

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

    public void setUniquePaths(List<Endpoint> endpoints) {

        Set<String> paths = new HashSet<>();

        for (Endpoint endpoint : endpoints) {
            paths.add(endpoint.getFullPath());
        }

        providedPaths.addAll(paths);
    }

    private void extractInfoFromDecorators(int i, CodeMemberDescriptor descriptor) {

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
}
