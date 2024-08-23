package it.unive.pylisa.libraries.pandas;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.dataframes.symbolic.ReadDataframe;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ReadUnknown
		extends
		it.unive.lisa.program.cfg.statement.Expression
		implements
		PluggableStatement {

	private Statement st;

	public ReadUnknown(
			CFG cfg,
			CodeLocation location) {
		super(cfg,
				location,
				PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference());
	}

	@Override
	public String toString() {
		return "read unknown";
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	public static ReadUnknown build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReadUnknown(cfg, location);
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), st);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		UnaryExpression read = new UnaryExpression(dftype,
				new PushAny(getCFG().getDescriptor().getUnit().getProgram().getTypes().getStringType(), location),
				new ReadDataframe(0), location);
		return PandasSemantics.createAndInitDataframe(entryState, read, st);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		return 0;
	}
}