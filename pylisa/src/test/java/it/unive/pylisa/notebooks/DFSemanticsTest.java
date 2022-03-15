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
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AccessOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ProjectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.NumberSlice;
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
					SideEffectAwareDataframeDomain,
					TypeEnvironment<InferredTypes>>,
			PointBasedHeap,
			SideEffectAwareDataframeDomain,
			TypeEnvironment<InferredTypes>> base;

	private final VariableRef df1;
	private final SourceCodeLocation readloc = new SourceCodeLocation("read", 0, 0);
	private final DataframeGraphDomain baseGraph = new DataframeGraphDomain(new ReadFromFile(readloc, fname));
	private final AllocationSite site;

	private final StatementStore<
			SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, SideEffectAwareDataframeDomain,
			TypeEnvironment<InferredTypes>> expressions;

	private final ModularWorstCaseAnalysis<
			SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
			PointBasedHeap, SideEffectAwareDataframeDomain,
			TypeEnvironment<InferredTypes>> interproc = new ModularWorstCaseAnalysis<>();

	public DFSemanticsTest() throws AnalysisSetupException, SemanticException {

		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		SideEffectAwareDataframeDomain df = new SideEffectAwareDataframeDomain();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<InferredTypes>(new InferredTypes());
		SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<
				InferredTypes>> entry = LiSAFactory.getInstance(SimpleAbstractState.class, heap, df, type);

		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> entryState = new AnalysisState<>(entry, new ExpressionSet<>());

		expressions = new StatementStore<>(entryState);

		StringLiteral lit = new StringLiteral(fakeCfg, loc, fname);
		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> state = lit.semantics(entryState, interproc, expressions);

		ReadCsv read = new ReadCsv(fakeCfg, readloc, lit);
		read.setOriginatingStatement(read);
		site = new AllocationSite(PandasDataframeType.INSTANCE, readloc.getCodeLocation(), null, false, readloc);

		df1 = new VariableRef(fakeCfg, loc, "df1", PandasDataframeType.REFERENCE);
		Assignment assignment = new Assignment(fakeCfg, loc, df1, read);
		base = assignment.semantics(state, interproc, expressions);
	}

	@Test
	public void testReadCsv() throws SemanticException, AnalysisSetupException {
		DataframeGraphDomain elem = new DataframeGraphDomain(new ReadFromFile(readloc, fname));
		DFOrConstant df = new DFOrConstant(elem);

		// the df is initialized is right
		assertEquals(df, base.getState().getValueState().getEnv().getState(site));
	}

	@Test
	public void testHead() throws SemanticException, AnalysisSetupException {
		SourceCodeLocation headloc = new SourceCodeLocation("head", 0, 0);

		DataframeGraphDomain elem1 = new DataframeGraphDomain(baseGraph, new AccessOperation<>(headloc, new NumberSlice(0, 5)));
		DFOrConstant df1 = new DFOrConstant(elem1);
		DataframeGraphDomain elem2 = new DataframeGraphDomain(baseGraph,
				new ProjectionOperation<>(headloc, new NumberSlice(0, 5)));
		DFOrConstant df2 = new DFOrConstant(elem2);

		AllocationSite headsite = new AllocationSite(PandasDataframeType.INSTANCE, headloc.getCodeLocation(), null,
				false,
				headloc);
		Head head = new Head(fakeCfg, headloc, this.df1, new Int32Literal(fakeCfg, loc, 5));
		head.setOriginatingStatement(head);

		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> actual = head.semantics(base, interproc, expressions);

		assertEquals(df1, actual.getState().getValueState().getEnv().getState(site));
		assertEquals(df2, actual.getState().getValueState().getEnv().getState(headsite));
	}
}
