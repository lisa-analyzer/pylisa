package it.unive.pylisa.libraries.fastapi;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.libraries.fastapi.models.Endpoint;
import it.unive.pylisa.libraries.fastapi.models.Method;
import it.unive.pylisa.libraries.fastapi.models.Param;
import lombok.experimental.UtilityClass;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.StreamSupport;

@UtilityClass
public class EndpointService {

    private final List<Endpoint> endpoints = new ArrayList<>();

    public List<Endpoint> gatherEndpoints(CheckTool tool, Unit unit) {

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
