package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.edge.ConsumeEdge;
import it.unive.pylisa.analysis.dataframes.edge.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.operations.Read;
import it.unive.pylisa.analysis.dataframes.symbolic.ReadDataframe;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class DataframeDomainTests {

	private static final ProgramPoint pp = new ProgramPoint() {

		@Override
		public CodeLocation getLocation() {
			return SyntheticLocation.INSTANCE;
		}

		@Override
		public CFG getCFG() {
			return null;
		}

		@Override
		public Program getProgram() {
			return program;
		}
	};
	
	private static final Program program = new Program(new PythonFeatures(), new PythonTypeSystem());
	private static final SourceCodeLocation loc1 = new SourceCodeLocation("test", 0, 1);
	private static final SourceCodeLocation loc2 = new SourceCodeLocation("test", 0, 2);

	private static final PyClassType dftype;

	static {
		LibrarySpecificationProvider.load(program);
		LibrarySpecificationProvider.importLibrary(program, LibrarySpecificationProvider.PANDAS);
		dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
	}
	
	@Test
	public void simpleTests() {
		DataframeGraphDomain dom = new DataframeGraphDomain();

		DataframeGraphDomain top = dom.top();
		assertTrue(top.isTop());
		assertTrue(top.constants.isTop());
		assertTrue(top.constStack.isTop());
		assertTrue(top.graph.isTop());
		assertTrue(top.l.isTop());
		assertTrue(top.v.isTop());
		assertTrue(top.vStack == DataframeGraphDomain.NO_IDS);

		DataframeGraphDomain bot = dom.bottom();
		assertTrue(bot.isBottom());
		assertTrue(bot.constants.isBottom());
		assertTrue(bot.constStack.isBottom());
		assertTrue(bot.graph.isBottom());
		assertTrue(bot.l.isBottom());
		assertTrue(bot.v.isBottom());
		assertTrue(bot.vStack == DataframeGraphDomain.NO_IDS);
	}

	@Test
	public void testGraphLeq() throws SemanticException {
		SourceCodeLocation loc1 = new SourceCodeLocation("F", 0, 0);
		SourceCodeLocation loc2 = new SourceCodeLocation("F", 0, 1);

		Constant c1 = new Constant(StringType.INSTANCE, "F", SyntheticLocation.INSTANCE);
		Constant c2 = new Constant(StringType.INSTANCE, "G", SyntheticLocation.INSTANCE);

		Read r1 = new Read(loc1, 0, new ConstantPropagation(c1));
		Read r2 = new Read(loc2, 0, new ConstantPropagation(c2));
		Read r3 = new Read(loc1, 0, new ConstantPropagation(c2));
		DataframeOperation lub = r1.lub(r3);

		DataframeForest f1 = new DataframeForest(false);
		f1.addNode(r1);
		f1.addNode(r2);
		f1.addEdge(new SimpleEdge(r1, r2));

		DataframeForest f2 = new DataframeForest(false);
		f2.addNode(r3);
		assertFalse(f1.lessOrEqual(f2));
		assertFalse(f2.lessOrEqual(f1));

		f1.addNode(r3);
		assertFalse(f1.lessOrEqual(f2));
		assertTrue(f2.lessOrEqual(f1));

		f1.addEdge(new ConsumeEdge(lub, r2));
		assertFalse(f1.lessOrEqual(f2));
		assertTrue(f2.lessOrEqual(f1));
	}

	@Test
	public void testGraphLub() throws SemanticException {
		SourceCodeLocation loc1 = new SourceCodeLocation("F", 0, 0);
		SourceCodeLocation loc2 = new SourceCodeLocation("F", 0, 1);
		SourceCodeLocation loc3 = new SourceCodeLocation("F", 0, 3);

		Constant c1 = new Constant(StringType.INSTANCE, "F", SyntheticLocation.INSTANCE);
		Constant c2 = new Constant(StringType.INSTANCE, "G", SyntheticLocation.INSTANCE);

		Read r1 = new Read(loc1, 0, new ConstantPropagation(c1));
		Read r2 = new Read(loc2, 0, new ConstantPropagation(c2));
		Read r3 = new Read(loc1, 0, new ConstantPropagation(c2));
		Read r4 = new Read(loc3, 0, new ConstantPropagation(c1));

		DataframeForest f1 = new DataframeForest(false);
		f1.addNode(r1);
		f1.addNode(r2);
		f1.addEdge(new SimpleEdge(r1, r2));

		DataframeForest f2 = new DataframeForest(false);
		f2.addNode(r3);
		f2.addNode(r4);
		f2.addEdge(new ConsumeEdge(r3, r4));

		assertFalse(f1.lessOrEqual(f2));
		assertFalse(f2.lessOrEqual(f1));

		DataframeForest lub = f1.lub(f2);
		DataframeOperation rlub = r1.lub(r3);
		assertEquals(3, lub.getNodesCount());
		assertTrue(lub.getNodes().contains(r2));
		assertTrue(lub.getNodes().contains(r4));
		assertTrue(lub.getNodes().contains(rlub));
		assertTrue(lub.getEdges().contains(new SimpleEdge(rlub, r2)));
		assertTrue(lub.getEdges().contains(new ConsumeEdge(rlub, r4)));
		assertTrue(f1.lessOrEqual(lub));
		assertTrue(f2.lessOrEqual(lub));
	}

	@Test
	public void testGraphAddNode() throws SemanticException {
		SourceCodeLocation loc1 = new SourceCodeLocation("F", 0, 0);
		SourceCodeLocation loc2 = new SourceCodeLocation("F", 0, 1);

		Constant c1 = new Constant(StringType.INSTANCE, "F", SyntheticLocation.INSTANCE);
		Constant c2 = new Constant(StringType.INSTANCE, "G", SyntheticLocation.INSTANCE);

		Read r1 = new Read(loc1, 0, new ConstantPropagation(c1));
		Read r2 = new Read(loc2, 0, new ConstantPropagation(c2));
		Read r3 = new Read(loc1, 0, new ConstantPropagation(c2));
		DataframeOperation lub = r1.lub(r3);

		SimpleEdge e1 = new SimpleEdge(r1, r2);

		DataframeForest f1 = new DataframeForest(false);
		f1.addNode(r1);
		f1.addNode(r2);
		f1.addEdge(e1);
		assertEquals(2, f1.getNodesCount());
		assertTrue(f1.getNodes().contains(r1));
		assertTrue(f1.getNodes().contains(r2));
		assertEquals(1, f1.getEdgesCount());
		assertTrue(f1.getEdges().contains(e1));

		f1.addNode(r3);
		assertEquals(2, f1.getNodesCount());
		assertTrue(f1.getNodes().contains(r2));
		assertFalse(f1.getNodes().contains(r1));
		assertFalse(f1.getNodes().contains(r3));
		assertTrue(f1.getNodes().contains(lub));
		assertEquals(1, f1.getEdgesCount());
		assertFalse(f1.getEdges().contains(e1));
		assertTrue(f1.getEdges().contains(new SimpleEdge(lub, r2)));
	}

	@Test
	public void testAccessKeys() throws SemanticException {
		
	}

	@Test
	public void testAssignToConstant() throws SemanticException {
		
	}

	@Test
	public void testAssignToSelection() throws SemanticException {
		
	}

	@Test
	public void testAxisConcatenation() throws SemanticException {
		
	}

	@Test
	public void testBinaryTransfrom() throws SemanticException {
		
	}

	@Test
	public void testColumnProjection() throws SemanticException {
		
	}

	@Test
	public void testCopyDataframe() throws SemanticException {
		
	}

	@Test
	public void testCreateDataframe() throws SemanticException {
		
	}

	@Test
	public void testDataframeProjection() throws SemanticException {
		
	}

	@Test
	public void testDropCols() throws SemanticException {
		
	}

	@Test
	public void testIterate() throws SemanticException {
		
	}

	@Test
	public void testJoinCols() throws SemanticException {
		
	}

	@Test
	public void testReadDataframe() throws SemanticException {
		UnaryExpression read = new UnaryExpression(dftype, 
				new Constant(StringType.INSTANCE, "source", loc1), 
				new ReadDataframe(0), loc2);
		DataframeGraphDomain dom = new DataframeGraphDomain();
		DataframeGraphDomain res = dom.smallStepSemantics(read, pp, null);
		assertEquals(1, res.graph.getNodesCount());
		assertEquals(0, res.graph.getEdgesCount());
		DataframeOperation node = res.graph.getNodes().iterator().next();
		assertTrue(node instanceof Read);
		assertFalse(((Read) node).getFile().isTop());
		assertFalse(((Read) node).getFile().isBottom());
		assertEquals("source", ((Read) node).getFile().getConstant());
	}

	@Test
	public void testRowProjection() throws SemanticException {
		
	}

	@Test
	public void testSeriesComparison() throws SemanticException {
		
	}

	@Test
	public void testUnaryReshape() throws SemanticException {
		
	}

	@Test
	public void testUnaryTransform() throws SemanticException {
		
	}
}
