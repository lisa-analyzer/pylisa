package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.DFOrConstant;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.operations.FilterNullRows;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.FilterNull;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class DFGraphTest {

	private final ProgramPoint fake;

	private final Variable df1, df2;

	private final SideEffectAwareDataframeDomain base;

	private final String fname;

	public DFGraphTest() throws SemanticException {
		SideEffectAwareDataframeDomain singleton = new SideEffectAwareDataframeDomain();

		fake = new ProgramPoint() {

			@Override
			public CodeLocation getLocation() {
				return null;
			}

			@Override
			public CFG getCFG() {
				return null;
			}
		};

		df1 = new Variable(PandasDataframeType.INSTANCE, "df1", SyntheticLocation.INSTANCE);
		df2 = new Variable(PandasDataframeType.INSTANCE, "df2", SyntheticLocation.INSTANCE);

		fname = "foo.csv";

		DataframeGraphDomain elem = new DataframeGraphDomain(new ReadFromFile(fname));
		DFOrConstant state = new DFOrConstant(elem);
		ValueEnvironment<DFOrConstant> env = singleton.getEnv().putState(df1, state);
//		TODO build a more elaborate element here
//		elem = new DataframeGraphDomain(fname, df2Set, df2PSet, df2Map, df2Interval);
//		state = new DFOrConstant(elem);
//		env = env.putState(df2, state);
		base = new SideEffectAwareDataframeDomain(env);
	}

	@Test
	public void testReadCsv() throws SemanticException {
		Constant filename = new Constant(StringType.INSTANCE, fname, SyntheticLocation.INSTANCE);
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, filename, ReadDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		SideEffectAwareDataframeDomain sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getEnv().getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(new ReadFromFile(fname));

		assertEquals(expected, stack);
	}

	@Test
	public void testFilterNullRows() throws SemanticException {
		Constant filename = new Constant(StringType.INSTANCE, fname, SyntheticLocation.INSTANCE);
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, df1, FilterNull.INSTANCE,
				SyntheticLocation.INSTANCE);
		SideEffectAwareDataframeDomain sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getEnv().getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getEnv().getState(df1).df(), new FilterNullRows());

		assertEquals(expected, stack);
	}

}
