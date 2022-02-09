package it.unive.pylisa.notebooks.structure;

import static org.junit.Assert.assertEquals;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.junit.Test;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
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
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.ProjectRows;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class NotebookStructureDomainTest {

	private final ProgramPoint fake;

	private final Variable df, df2;

	private final SideEffectAwareDataframeDomain<DataframeStructureDomain,
			SingleDataframe> base;

	private final String fname;
	private final ColumnMapping df1Map, df2Map;
	private final ColumnSet df1Set, df2Set, df1PSet, df2PSet;
	private final Interval df1Interval, df2Interval;

	public NotebookStructureDomainTest() throws SemanticException {
		SideEffectAwareDataframeDomain<DataframeStructureDomain,
				SingleDataframe> singleton = new SideEffectAwareDataframeDomain<>(new DataframeStructureDomain());

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
		df2 = new Variable(PandasDataframeType.INSTANCE, "df2", SyntheticLocation.INSTANCE);

		fname = "foo.csv";
		df1Map = new ColumnMapping().top();
		df1Set = df1PSet = new ColumnSet().top();
		df1Interval = new Interval().top();
		df2Set = new ColumnSet(Set.of("c1", "c2", "c3", "c4"));
		df2PSet = new ColumnSet(Set.of("s1", "s2", "s3"));
		df2Map = new ColumnMapping(Map.of(0, new ColumnSet("c1"), 3, new ColumnSet(Set.of("c2", "s3"))));
		df2Interval = new Interval(15, 38);

		ConstantPropagation bot = new ConstantPropagation().bottom();
		SingleDataframe elem = new SingleDataframe(fname, df1Set, df1Set, df1Map, df1Interval);
		DataframeStructureDomain state = new DataframeStructureDomain(elem, bot);
		ValueEnvironment<DataframeStructureDomain> env = singleton.getEnv().putState(df, state);
		elem = new SingleDataframe(fname, df2Set, df2PSet, df2Map, df2Interval);
		state = new DataframeStructureDomain(elem, bot);
		env = env.putState(df2, state);
		base = new SideEffectAwareDataframeDomain<>(env);
	}

	@Test
	public void testReadCsv() throws SemanticException {
		String fname = "foo.csv";
		Constant filename = new Constant(StringType.INSTANCE, fname, SyntheticLocation.INSTANCE);
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, filename, ReadDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		SideEffectAwareDataframeDomain<DataframeStructureDomain,
				SingleDataframe> sss = base.smallStepSemantics(unary, fake);
		SingleDataframe stack = sss.getEnv().getValueOnStack().left;
		SingleDataframe expected = new SingleDataframe(fname, df1Set, df1PSet, df1Map, df1Interval);

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
		SingleDataframe expected = new SingleDataframe(fname, df1Set, df1PSet, df1Map, new Interval(0, 10));

		assertEquals(expected, stack);

		ternary = new TernaryExpression(PandasDataframeType.INSTANCE, df2, zero, ten,
				ProjectRows.INSTANCE, SyntheticLocation.INSTANCE);

		sss = base.smallStepSemantics(ternary, fake);
		stack = sss.getEnv().getValueOnStack().left;
		expected = new SingleDataframe(fname, df2Set, df2PSet, df2Map, new Interval(0, 10));

		assertEquals(expected, stack);
	}

	@Test
	public void testColumnAccess() throws SemanticException {
		Constant col = new Constant(StringType.INSTANCE, "foo", SyntheticLocation.INSTANCE);
		BinaryExpression binary = new BinaryExpression(PandasDataframeType.INSTANCE, df, col,
				ColumnAccess.INSTANCE, SyntheticLocation.INSTANCE);

		SideEffectAwareDataframeDomain<DataframeStructureDomain,
				SingleDataframe> sss = base.smallStepSemantics(binary, fake);
		SingleDataframe stack = sss.getEnv().getValueOnStack().left;
		SingleDataframe expected = new SingleDataframe(fname, new ColumnSet("foo"), df1PSet, df1Map, df1Interval);

		assertEquals(expected, stack);

		binary = new BinaryExpression(PandasDataframeType.INSTANCE, df2, col,
				ColumnAccess.INSTANCE, SyntheticLocation.INSTANCE);

		sss = base.smallStepSemantics(binary, fake);
		stack = sss.getEnv().getValueOnStack().left;
		Set<String> names = new HashSet<>(df2Set.elements());
		names.add("foo");
		expected = new SingleDataframe(fname, new ColumnSet(names), df2PSet, df2Map, df2Interval);

		assertEquals(expected, stack);
	}
}
