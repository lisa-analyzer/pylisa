package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import java.util.Collection;

/**
 * A cfg of a python program. CFGs are compared by identity, but their hash code
 * is computed from their name, unit and location (see
 * {@link #hashCode(CodeMember)}): the analysis keeps them in hash maps and
 * sets, whose iteration order must not depend on the memory addresses of the
 * objects (that change, e.g., with the analyses run before in the same virtual
 * machine), otherwise the results of the analysis would not be deterministic.
 */
public class PyCFG extends CFG {

	private Integer hash;

	public PyCFG(
			CodeMemberDescriptor descriptor) {
		super(descriptor);
	}

	public PyCFG(
			CodeMemberDescriptor descriptor,
			Collection<Statement> entrypoints,
			NodeList<CFG, Statement, Edge> list) {
		super(descriptor, entrypoints, list);
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj;
	}

	@Override
	public int hashCode() {
		if (hash == null)
			hash = hashCode(this);
		return hash;
	}

	/**
	 * A hash code of the code member that does not depend on its memory
	 * address: it is computed from the parts of its descriptor that never
	 * change (its name, the name of its unit and its location, as strings).
	 *
	 * @param member the code member
	 *
	 * @return the hash code
	 */
	public static int hashCode(
			CodeMember member) {
		CodeMemberDescriptor descriptor = member.getDescriptor();
		int result = descriptor.getName().hashCode();
		result = 31 * result + (descriptor.getUnit() == null ? 0 : descriptor.getUnit().getName().hashCode());
		result = 31 * result + String.valueOf(descriptor.getLocation()).hashCode();
		return 31 * result + descriptor.getFormals().length;
	}
}
