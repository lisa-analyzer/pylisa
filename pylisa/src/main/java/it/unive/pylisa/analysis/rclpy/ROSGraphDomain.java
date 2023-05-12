package it.unive.pylisa.analysis.rclpy;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.CollectingMapLattice;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.NodeId;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.checks.semantics.ROSComputationGraphDumper;
import it.unive.pylisa.models.rclpy.RosComputationalGraph;

import java.util.function.Predicate;

public class ROSGraphDomain extends BaseNonRelationalValueDomain<ROSGraphDomain> {
    private RosComputationalGraph ROSGraph;
    private ValueEnvironment<ConstantPropagation> constants;

    public ROSGraphDomain() {
        this.constants = new ValueEnvironment<ConstantPropagation>;
        this.ROSGraph = new RosComputationalGraph();

    }
    private ROSGraphDomain(
            ValueEnvironment<ConstantPropagation> constants,
            RosComputationalGraph ROSGraph
            ) {
        this.constants = constants;
        this.ROSGraph = ROSGraph;
    }
    public RosComputationalGraph getROSGraph() {
        return ROSGraph;
    }

    @Override
    public ROSGraphDomain lubAux(ROSGraphDomain other) throws SemanticException {
        return null;
    }

    @Override
    public boolean lessOrEqualAux(ROSGraphDomain other) throws SemanticException {
        return false;
    }

    @Override
    public boolean equals(Object obj) {
        return false;
    }

    @Override
    public int hashCode() {
        return 0;
    }

    @Override
    public DomainRepresentation representation() {
        return null;
    }

    @Override
    public ROSGraphDomain top() {
        return null;
    }

    @Override
    public ROSGraphDomain bottom() {
        return null;
    }
}
