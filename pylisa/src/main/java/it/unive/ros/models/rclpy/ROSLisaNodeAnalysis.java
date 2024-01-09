package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;

public class ROSLisaNodeAnalysis<A extends AbstractState<A>> extends ROSLisaAnalysis {

    InterproceduralAnalysis<A> interproceduralAnalysis;
    public ROSLisaNodeAnalysis(SymbolicExpression symbolicExpression, Statement statement, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState) {
        super(symbolicExpression, statement, analysisState);
    }
    public ROSLisaNodeAnalysis(SymbolicExpression symbolicExpression, Statement statement, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, InterproceduralAnalysis<A> interproceduralAnalysis) {
        this(symbolicExpression, statement, analysisState);
        this.interproceduralAnalysis = interproceduralAnalysis;
    }
    public InterproceduralAnalysis<A> interproceduralAnalysis() {
        return interproceduralAnalysis;
    }
}
