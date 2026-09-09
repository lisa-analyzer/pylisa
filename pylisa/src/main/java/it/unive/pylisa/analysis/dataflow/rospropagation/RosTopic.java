package it.unive.pylisa.analysis.dataflow.rospropagation;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.dataflow.DataflowElement;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Collection;
import java.util.Collections;
import java.util.Objects;

public class RosTopic
		implements
		DataflowElement<RosTopic> {
	// instances of this class will go inside a Collection (override equals and
	// hashCode function are necessary)
	private final Identifier id;
	private final ProgramPoint programPoint;
	// private String topicName;
	// private String msgType;
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
		// this.topicName = topicName;
		// this.msgType = msgType;
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
	public RosTopic replaceIdentifier(
			Identifier source,
			Identifier target) {
		if (id == null || !id.equals(source))
			return this;
		return new RosTopic(target, programPoint, "", "");
	}

	@Override
	public StructuredRepresentation representation() {
		return new StringRepresentation(id);
	}

	@Override
	public RosTopic pushScope(
			ScopeToken scopeToken,
			ProgramPoint pp)
			throws SemanticException {
		return this;
	}

	@Override
	public RosTopic popScope(
			ScopeToken scopeToken,
			ProgramPoint pp)
			throws SemanticException {
		return this;
	}
}
