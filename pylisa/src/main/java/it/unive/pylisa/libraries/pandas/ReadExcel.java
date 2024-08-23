package it.unive.pylisa.libraries.pandas;

import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.analysis.dataframes.symbolic.ReadDataframe;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.StringAdd;

public class ReadExcel
		extends
		it.unive.lisa.program.cfg.statement.BinaryExpression
		implements
		PluggableStatement {

	private static final Logger LOG = LogManager.getLogger(ReadExcel.class);

	private Statement st;

	public ReadExcel(
			CFG cfg,
			CodeLocation location,
			Expression origin,
			Expression sheets) {
		super(cfg,
				location,
				"read_excel",
				PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF).getReference(),
				origin,
				sheets);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	public static ReadExcel build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ReadExcel(cfg, location, exprs[0], exprs[1]);
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type> rts = state.getState().getRuntimeTypesOf(right, st, state.getState());
		if (rts.contains(PyClassType.lookup(LibrarySpecificationProvider.LIST))) {
			LOG.warn("Reading multiple excel sheets not supported, will lose knowledge of origin");
			return ReadUnknown.build(getCFG(), getLocation(), null)
					.forwardSemantics(state, interprocedural, expressions);
		}
		CodeLocation location = getLocation();
		PyClassType dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		StringType stringType = getCFG().getDescriptor().getUnit().getProgram().getTypes().getStringType();

		BinaryExpression arg = new BinaryExpression(
				stringType, 
				left, 
				new BinaryExpression(
						stringType, 
						new Constant(stringType, "::", location), 
						right,
						StringAdd.INSTANCE, 
						location), 
				StringAdd.INSTANCE, 
				location);
		
		UnaryExpression read = new UnaryExpression(dftype, arg, new ReadDataframe(0), location);
		return PandasSemantics.createAndInitDataframe(state, read, st);
	}
}