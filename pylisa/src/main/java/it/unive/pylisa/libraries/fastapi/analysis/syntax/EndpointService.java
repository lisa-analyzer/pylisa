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
import it.unive.pylisa.libraries.fastapi.definitions.GroupBy;
import it.unive.pylisa.libraries.fastapi.definitions.Method;
import it.unive.pylisa.libraries.fastapi.definitions.Param;
import it.unive.pylisa.libraries.fastapi.definitions.Role;
import it.unive.pylisa.libraries.fastapi.helpers.TextHelper;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;
import lombok.experimental.UtilityClass;
import org.apache.http.client.utils.URIBuilder;

@UtilityClass
public class EndpointService {

	public List<Endpoint> endpoints = new ArrayList<>();

	public List<Endpoint> gatherProviderEndpoints(
			Unit unit) {

		int index = 0;
		for (CodeMember member : unit.getCodeMembers()) {

			CodeMemberDescriptor descriptor = member.getDescriptor();
			EndpointService.extractInfoFromDecorators(index, descriptor);

			for (Parameter parameter : descriptor.getFormals()) {
				specifyMethodsArguments(index, (PyParameter) parameter);
			}
		}

		String sourceName = TextHelper.getFilenameFromUnit(unit);

		for (Endpoint endpoint : endpoints) {
			endpoint.setBelongs(sourceName);
		}

		List<Endpoint> result = endpoints;
		endpoints = new ArrayList<>();

		return result;
	}

	public Endpoint gatherConsumerEndpoint(
			Statement node) {

		if (node instanceof PyStringLiteral pyStringLiteral) {

			if (pyStringLiteral.getParentStatement() instanceof UnresolvedCall parent) {

				if (Arrays.stream(parent.getSubExpressions()).toList().get(0) instanceof VariableRef variableRef) {

					if (variableRef.getName().equals("requests")) {

						PyStringLiteral endpointInfo = (PyStringLiteral) Arrays.stream(parent.getSubExpressions())
								.toList().get(1);

						URIBuilder url = null;

						try {
							url = new URIBuilder(endpointInfo.getValue());
						} catch (Exception ignored) {
						}

						Endpoint endpoint = new Endpoint();
						endpoint.setMethod(Method.specify(parent.getConstructName()));
						endpoint.setRole(Role.CONSUMER);
						endpoint.setFullPath(url.getPath());
						endpoint.setCodeLocation(TextHelper.getCodeline(endpointInfo.getLocation().getCodeLocation()));

						return endpoint;
					}
				}
			}
		}

		return null;
	}

	private void specifyMethodsArguments(
			int i,
			PyParameter parameter) {

		Param param = new Param(parameter.getTypeHint(), parameter.getName());
		endpoints.get(i).getMethodPathVariable().add(param);
	}

	public AnnotationMember extractDecoratorByMethod(
			CFG graph,
			Method method) {

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

	private void extractInfoFromDecorators(
			int i,
			CodeMemberDescriptor descriptor) {

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

	public Map<String, List<Endpoint>> groupEndpoints(
			List<Endpoint> endpoints,
			GroupBy by) {

		return switch (by) {
		case ROLE_CONSUMER -> endpoints.stream().filter(endpoint -> endpoint.getRole() == Role.CONSUMER)
				.collect(Collectors.groupingBy(endpoint -> endpoint.getRole().toString(), Collectors.toList()));
		case ROLE_PROVIDER -> endpoints.stream().filter(endpoint -> endpoint.getRole() == Role.PROVIDER)
				.collect(Collectors.groupingBy(endpoint -> endpoint.getRole().toString(), Collectors.toList()));
		default -> endpoints.stream().collect(Collectors.groupingBy(Endpoint::getBelongs, Collectors.toList()));
		};
	}
}
