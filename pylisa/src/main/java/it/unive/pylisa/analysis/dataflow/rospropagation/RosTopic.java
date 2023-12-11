package it.unive.pylisa.analysis.dataflow.rospropagation;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.dataflow.DataflowElement;
import it.unive.lisa.analysis.dataflow.PossibleDataflowDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Objects;

public class RosTopic
		implements
		DataflowElement<
				PossibleDataflowDomain<RosTopic>, // dataflow domain that
													// we want
				RosTopic> {
	// instances of this class will go inside a Collection (override equals and
	// hashCode function are necessary)
	private final Identifier id;
	private final ProgramPoint programPoint;
	private String topicName;
	private String msgType;
	// private List<String> publisher;
	// private List<String> subscriber;

	public RosTopic() {
		this(null, null, "", "");
	}

	public RosTopic(
			Identifier id,
			ProgramPoint programPoint,
			String topicName,
			String msgType) {
		this.id = id;
		this.programPoint = programPoint;
		this.topicName = topicName;
		this.msgType = msgType;
	}

	@Override
	public boolean equals(
			Object o) {
		if (this == o)
			return true;
		if (o == null || getClass() != o.getClass())
			return false;

		RosTopic rosTopic = (RosTopic) o;

		if (!Objects.equals(id, rosTopic.id))
			return false;
		return Objects.equals(programPoint, rosTopic.programPoint);
	}

	@Override
	public int hashCode() {
		int result = id != null ? id.hashCode() : 0;
		result = 31 * result + (programPoint != null ? programPoint.hashCode() : 0);
		return result;
	}

	@Override
	public Collection<Identifier> getInvolvedIdentifiers() {
		// get all indentifier that are involved in this dataflow element.
		return Collections.singleton(id);
	}

	@Override
	public Collection<RosTopic> gen(
			Identifier identifier,
			ValueExpression valueExpression,
			ProgramPoint programPoint,
			PossibleDataflowDomain<RosTopic> rosTopicPossibleForwardDataflowDomain)
			throws SemanticException {
		// System.out.println("GEN from assignment (identifier = expression)");
		// System.out.println("value exp. -> " + valueExpression);
		// System.out.println("identifier -> " + identifier);
		// System.out.println("programpoint -> " + programPoint);

		return new HashSet<RosTopic>();
	}

	@Override
	public Collection<RosTopic> gen(
			ValueExpression valueExpression,
			ProgramPoint programPoint,
			PossibleDataflowDomain<RosTopic> rosTopicPossibleForwardDataflowDomain)
			throws SemanticException {
		// System.out.println("GEN from evaluation (non assigning expression)");
		// System.out.println("value exp. -> " + valueExpression);
		// System.out.println("identifier -> " + identifier);
		// System.out.println("programpoint -> " + programPoint);
		if (programPoint instanceof OpenCall) {
			if (Objects.equals(((OpenCall) programPoint).getTargetName(), "create_subscription")) {
				// topic name and message type used by the publisher and
				// subscriber must match to allow them to communicate.
				System.out.println("FOUND CREATE_SUBSCRIPTION");
				System.out.println("TOPIC NAME -> " + ((OpenCall) programPoint).getSubExpressions()[2]);
				System.out.println("MESSAGE TYPE -> " + ((OpenCall) programPoint).getSubExpressions()[1]);
				return Collections
						.singleton(new RosTopic(null, null, ((OpenCall) programPoint).getSubExpressions()[2].toString(),
								((OpenCall) programPoint).getSubExpressions()[1].toString()));

				// Build RosTopic

			}
			if (Objects.equals(((OpenCall) programPoint).getTargetName(), "create_publisher")) {
				System.out.println("FOUND CREATE_PUBLISHER");
				System.out.println("TOPIC NAME -> " + ((OpenCall) programPoint).getSubExpressions()[2]);
				System.out.println("MESSAGE TYPE -> " + ((OpenCall) programPoint).getSubExpressions()[1]);
				return Collections
						.singleton(new RosTopic(null, null, ((OpenCall) programPoint).getSubExpressions()[2].toString(),
								((OpenCall) programPoint).getSubExpressions()[1].toString()));

				// Build RosTopic

			}
		}
		return new HashSet<RosTopic>();
	}

	@Override
	public Collection<RosTopic> kill(
			Identifier identifier,
			ValueExpression valueExpression,
			ProgramPoint programPoint,
			PossibleDataflowDomain<RosTopic> rosTopicPossibleForwardDataflowDomain)
			throws SemanticException {
		// System.out.println("KILL from assignment (identifier = expression)");
		// System.out.println("value exp. -> " + valueExpression);
		// System.out.println("identifier -> " + identifier);
		// System.out.println("programpoint -> " + programPoint);
		return new HashSet<RosTopic>();
	}

	@Override
	public Collection<RosTopic> kill(
			ValueExpression valueExpression,
			ProgramPoint programPoint,
			PossibleDataflowDomain<RosTopic> rosTopicPossibleForwardDataflowDomain)
			throws SemanticException {
		// System.out.println("KILL from evaluation (non assigning
		// expression)");
		// System.out.println("value exp. -> " + valueExpression);
		// System.out.println("identifier -> " + identifier);
		// System.out.println("programpoint -> " + programPoint);
		return new HashSet<RosTopic>();
	}

	@Override
	public StructuredRepresentation representation() {
		return new StringRepresentation(id);
	}

	@Override
	public RosTopic pushScope(
			ScopeToken scopeToken)
			throws SemanticException {
		return this;
	}

	@Override
	public RosTopic popScope(
			ScopeToken scopeToken)
			throws SemanticException {
		return this;
	}
}
