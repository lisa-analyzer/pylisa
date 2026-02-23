package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Statement;

public class PyCFG extends CFG {
	public PyCFG(
			CodeMemberDescriptor descriptor) {
		super(descriptor);
	}

	public void addNodeIfNotPresent(
			Statement n) {
		if (!this.list.getNodes().contains(n))
			this.addNode(n);
	}

	public void addNodeIfNotPresent(
			Statement n,
			boolean entrypoint) {
		this.addNodeIfNotPresent(n);
		if (entrypoint)
			this.entrypoints.add(n);
	}
}
