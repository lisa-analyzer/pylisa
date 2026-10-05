package it.unive.pylisa.cfg;

import java.util.Collection;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;

public class PyCFG extends CFG {

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
}
