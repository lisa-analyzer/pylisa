package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;

public class ROSLisaNodeAnalysis<A extends AbstractLattice<A>> extends ROSLisaAnalysis {

	InterproceduralAnalysis<A, ?> interproceduralAnalysis;

	public ROSLisaNodeAnalysis(
			SymbolicExpression symbolicExpression,
			Statement statement,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState) {
		super(symbolicExpression, statement, analysisState);
	}

	public ROSLisaNodeAnalysis(
			SymbolicExpression symbolicExpression,
			Statement statement,
			AnalysisState<SimpleAbstractState<HeapEnvironment<AllocationSites>, ValueEnvironment<ConstantPropagation>,
					TypeEnvironment<TypeSet>>> analysisState,
			InterproceduralAnalysis<A, ?> interproceduralAnalysis) {
		this(symbolicExpression, statement, analysisState);
		this.interproceduralAnalysis = interproceduralAnalysis;
	}

	public InterproceduralAnalysis<A, ?> interproceduralAnalysis() {
		return interproceduralAnalysis;
	}
}
