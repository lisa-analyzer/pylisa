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
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.analysis.dataframes.DFOrConstant;
import it.unive.pylisa.analysis.dataframes.SideEffectAwareDataframeDomain;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.libraries.pandas.ReadCsv;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;

@SuppressWarnings("unchecked")
public class DFSemanticsTest {

	private final CodeLocation loc = SyntheticLocation.INSTANCE;
	private final CompilationUnit fakeUnit = new CompilationUnit(loc, "fake", false);
	private final CFG fakeCfg = new CFG(new CFGDescriptor(loc, fakeUnit, false, "fake"));

	@Test
	public void testReadCsv() throws SemanticException, AnalysisSetupException {
		FieldSensitivePointBasedHeap heap = new FieldSensitivePointBasedHeap();
		SideEffectAwareDataframeDomain sea = new SideEffectAwareDataframeDomain();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<InferredTypes>(new InferredTypes());
		SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<
				InferredTypes>> entry = LiSAFactory.getInstance(SimpleAbstractState.class, heap, sea, type);

		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> entryState = new AnalysisState<>(entry, new ExpressionSet<>());

		StatementStore<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> expressions = new StatementStore<>(entryState);

		ModularWorstCaseAnalysis<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> interproc = new ModularWorstCaseAnalysis<>();

		String fname = "foo.csv";
		StringLiteral lit = new StringLiteral(fakeCfg, loc, fname);
		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> state = lit.semantics(entryState, interproc, expressions);

		ReadCsv read = new ReadCsv(fakeCfg, loc, lit);
		AnalysisState<
				SimpleAbstractState<PointBasedHeap, SideEffectAwareDataframeDomain, TypeEnvironment<InferredTypes>>,
				PointBasedHeap, SideEffectAwareDataframeDomain,
				TypeEnvironment<InferredTypes>> actual = read.semantics(state, interproc, expressions);

		DataframeGraphDomain elem = new DataframeGraphDomain(new ReadFromFile(fname));
		DFOrConstant df = new DFOrConstant(elem);
		AllocationSite site = new AllocationSite(PandasDataframeType.INSTANCE, loc.getCodeLocation(), null, false, loc);

		SymbolicExpression stack = new HeapReference(PandasDataframeType.REFERENCE,
				new HeapAllocation(PandasDataframeType.INSTANCE, loc), loc);

		InferredTypes expTypes = new InferredTypes().eval(new Constant(PandasDataframeType.INSTANCE, null, loc), type,
				read);

		// heap is empty
		assertEquals(heap, actual.getState().getHeapState());
		// type of the location where the df is initialized is right
		assertEquals(expTypes, actual.getState().getTypeState().getState(site));
		// the df is initialized is right
		assertEquals(df, actual.getState().getValueState().getEnv().getState(site));
		// the ref is left on the stack
		assertEquals(new ExpressionSet<>(stack), actual.getComputedExpressions());

	}
}
