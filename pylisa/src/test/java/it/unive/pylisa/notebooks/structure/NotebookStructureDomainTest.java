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
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import it.unive.pylisa.analysis.dataframes.structure.ColumnMapping;
import it.unive.pylisa.analysis.dataframes.structure.ColumnSet;
import it.unive.pylisa.analysis.dataframes.structure.DataframeStructureDomain;
import it.unive.pylisa.analysis.dataframes.structure.SingleDataframe;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class NotebookStructureDomainTest {

	private final SideEffectAwareDataframeDomain<DataframeStructureDomain,
			SingleDataframe> singleton = new SideEffectAwareDataframeDomain<>(new DataframeStructureDomain());

	private final ProgramPoint fake = new ProgramPoint() {

		@Override
		public CodeLocation getLocation() {
			return null;
		}

		@Override
		public CFG getCFG() {
			return null;
		}
	};

	private final ColumnMapping topMapping = new ColumnMapping().top();
	private final ColumnSet topSet = new ColumnSet().top();
	private final Interval topInterval = new Interval().top();

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
}
