package it.unive.pylisa.notebooks.structure;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.common.Int32;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import it.unive.pylisa.analysis.dataframes.structure.ColumnMapping;
import it.unive.pylisa.analysis.dataframes.structure.ColumnSet;
import it.unive.pylisa.analysis.dataframes.structure.DataframeStructureDomain;
import it.unive.pylisa.analysis.dataframes.structure.SingleDataframe;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.ProjectRows;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class NotebookStructureDomainTest {

	private final SideEffectAwareDataframeDomain<DataframeStructureDomain,
			SingleDataframe> singleton;

	private final ProgramPoint fake;

	private final Variable df;

	private final SideEffectAwareDataframeDomain<DataframeStructureDomain,
			SingleDataframe> base;

	private final String fname;
	private final ColumnMapping topMapping;
	private final ColumnSet topSet;
	private final Interval topInterval;

	public NotebookStructureDomainTest() throws SemanticException {
		singleton = new SideEffectAwareDataframeDomain<>(new DataframeStructureDomain());

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

		df = new Variable(PandasDataframeType.INSTANCE, "df", SyntheticLocation.INSTANCE);

		topMapping = new ColumnMapping().top();
		topSet = new ColumnSet().top();
		topInterval = new Interval().top();

		fname = "foo.csv";
		SingleDataframe dfElement = new SingleDataframe(fname, topSet, topSet, topMapping, topInterval);
		base = new SideEffectAwareDataframeDomain<>(singleton.getEnv().putState(df,
				new DataframeStructureDomain(dfElement, new ConstantPropagation().bottom())));
	}

	@Test
	public void testReadCsv() throws SemanticException {
		String fname = "foo.csv";
		Constant filename = new Constant(StringType.INSTANCE, fname, SyntheticLocation.INSTANCE);
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, filename, ReadDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		SideEffectAwareDataframeDomain<DataframeStructureDomain,
				SingleDataframe> sss = singleton.smallStepSemantics(unary, fake);
		SingleDataframe stack = sss.getEnv().getValueOnStack().left;
		SingleDataframe expected = new SingleDataframe(fname, topSet, topSet, topMapping, topInterval);

		assertEquals(expected, stack);
	}

	@Test
	public void testHead() throws SemanticException {
		Constant zero = new Constant(Int32.INSTANCE, 0, SyntheticLocation.INSTANCE);
		Constant ten = new Constant(Int32.INSTANCE, 10, SyntheticLocation.INSTANCE);
		TernaryExpression ternary = new TernaryExpression(PandasDataframeType.INSTANCE, df, zero, ten,
				ProjectRows.INSTANCE, SyntheticLocation.INSTANCE);

		SideEffectAwareDataframeDomain<DataframeStructureDomain,
				SingleDataframe> sss = base.smallStepSemantics(ternary, fake);
		SingleDataframe stack = sss.getEnv().getValueOnStack().left;
		SingleDataframe expected = new SingleDataframe(fname, topSet, topSet, topMapping, new Interval(0, 10));

		assertEquals(expected, stack);
	}
}
