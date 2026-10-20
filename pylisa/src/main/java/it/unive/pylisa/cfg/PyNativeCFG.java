package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.NaryExpression;

/**
 * A native function of the libraries. As for {@link PyCFG}, native functions
 * are compared by identity, but their hash code does not depend on their memory
 * address, so that the analysis is deterministic.
 */
public class PyNativeCFG extends NativeCFG {

	private Integer hash;

	/**
	 * Builds the native function.
	 *
	 * @param descriptor the descriptor of the function
	 * @param construct  the class implementing its semantics
	 */
	public PyNativeCFG(
			CodeMemberDescriptor descriptor,
			Class<? extends NaryExpression> construct) {
		super(descriptor, construct);
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj;
	}

	@Override
	public int hashCode() {
		if (hash == null)
			hash = PyCFG.hashCode(this);
		return hash;
	}
}
