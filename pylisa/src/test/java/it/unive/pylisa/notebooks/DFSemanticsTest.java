package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSAFactory;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.ModularWorstCaseAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.pylisa.analysis.dataframes.DFOrConstant;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.RowAccess;
import it.unive.pylisa.analysis.dataframes.transformation.operations.RowProjection;
import it.unive.pylisa.libraries.pandas.Head;
import it.unive.pylisa.libraries.pandas.ReadCsv;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

@SuppressWarnings("unchecked")
public class DFSemanticsTest {

	private final CodeLocation loc = SyntheticLocation.INSTANCE;
	private final CompilationUnit fakeUnit = new CompilationUnit(loc, "fake", false);
	private final CFG fakeCfg = new CFG(new CFGDescriptor(loc, fakeUnit, false, "fake"));

	private final String fname = "foo.csv";

	private final AnalysisState<
			SimpleAbstractState<
					PointBasedHeap,
					ValueEnvironment<DFOrConstant>,
					TypeEnvironment<InferredTypes>>,
			PointBasedHeap,
			ValueEnvironment<DFOrConstant>,
			TypeEnvironment<InferredTypes>> base;

	private final VariableRef df1;
	private final DataframeGraphDomain baseGraph = new DataframeGraphDomain(new ReadFromFile(fname));
	private final AllocationSite site;

	private final StatementStore<
			SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, ValueEnvironment<DFOrConstant>,
			TypeEnvironment<InferredTypes>> expressions;

	private final ModularWorstCaseAnalysis<
			SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, ValueEnvironment<DFOrConstant>,
			TypeEnvironment<InferredTypes>> interproc = new ModularWorstCaseAnalysis<>();

	public DFSemanticsTest() throws AnalysisSetupException, SemanticException {

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		ValueEnvironment<DFOrConstant> df = new ValueEnvironment<>(new DFOrConstant());
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<InferredTypes>(new InferredTypes());
		SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<
				InferredTypes>> entry = LiSAFactory.getInstance(SimpleAbstractState.class, heap, df, type);

		AnalysisState<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, ValueEnvironment<DFOrConstant>,
				TypeEnvironment<InferredTypes>> entryState = new AnalysisState<>(entry, new ExpressionSet<>());

		expressions = new StatementStore<>(entryState);

		StringLiteral lit = new StringLiteral(fakeCfg, loc, fname);
		AnalysisState<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, ValueEnvironment<DFOrConstant>,
				TypeEnvironment<InferredTypes>> state = lit.semantics(entryState, interproc, expressions);

		SourceCodeLocation readloc = new SourceCodeLocation("read", 0, 0);
		ReadCsv read = new ReadCsv(fakeCfg, readloc, lit);
		site = new AllocationSite(PandasDataframeType.INSTANCE, readloc.getCodeLocation(), null, false, readloc);

		df1 = new VariableRef(fakeCfg, loc, "df1", PandasDataframeType.REFERENCE);
		Assignment assignment = new Assignment(fakeCfg, loc, df1, read);
		base = assignment.semantics(state, interproc, expressions);
	}

	@Test
	public void testReadCsv() throws SemanticException, AnalysisSetupException {
		DataframeGraphDomain elem = new DataframeGraphDomain(new ReadFromFile(fname));
		DFOrConstant df = new DFOrConstant(elem);

		// the df is initialized is right
		assertEquals(df, base.getState().getValueState().getState(site));
	}

	@Test
	public void testHead() throws SemanticException, AnalysisSetupException {
		DataframeGraphDomain elem1 = new DataframeGraphDomain(baseGraph, new RowAccess(new Interval(0, 5)));
		DFOrConstant df1 = new DFOrConstant(elem1);
		DataframeGraphDomain elem2 = new DataframeGraphDomain(baseGraph, new RowProjection(new Interval(0, 5)));
		DFOrConstant df2 = new DFOrConstant(elem2);

		SourceCodeLocation headloc = new SourceCodeLocation("head", 0, 0);
		AllocationSite headsite = new AllocationSite(PandasDataframeType.INSTANCE, headloc.getCodeLocation(), null,
				false,
				headloc);
		Head head = new Head(fakeCfg, headloc, this.df1, new Int32Literal(fakeCfg, loc, 5));

		AnalysisState<
				SimpleAbstractState<PointBasedHeap, ValueEnvironment<DFOrConstant>, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, ValueEnvironment<DFOrConstant>,
				TypeEnvironment<InferredTypes>> actual = head.semantics(base, interproc, expressions);

		assertEquals(df1, actual.getState().getValueState().getState(site));
		assertEquals(df2, actual.getState().getValueState().getState(headsite));
	}
}
