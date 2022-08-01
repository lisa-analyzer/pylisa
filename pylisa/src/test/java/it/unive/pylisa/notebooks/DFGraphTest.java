package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import org.junit.Test;
import org.reflections.Reflections;
import org.reflections.scanners.SubTypesScanner;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.common.Int32;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.DFOrConstant;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.Names;
import it.unive.pylisa.analysis.dataframes.transformation.graph.AssignEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AccessOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AssignDataframe;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AssignValue;
import it.unive.pylisa.analysis.dataframes.transformation.operations.BooleanComparison;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Concat;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DropColumns;
import it.unive.pylisa.analysis.dataframes.transformation.operations.FilterNullAxis;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ProjectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.SelectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Transform;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.AtomicBooleanSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.ColumnListSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.NumberSlice;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.SliceConstant;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRows;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRowsColumns;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation.Kind;
import it.unive.pylisa.symbolic.operators.dataframes.AxisConcatenation;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnAccess;
import it.unive.pylisa.symbolic.operators.dataframes.ComparisonOperator;
import it.unive.pylisa.symbolic.operators.dataframes.DropCols;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull.Axis;
import it.unive.pylisa.symbolic.operators.dataframes.JoinCols;
import it.unive.pylisa.symbolic.operators.dataframes.PandasSeriesComparison;
import it.unive.pylisa.symbolic.operators.dataframes.PopSelection;
import it.unive.pylisa.symbolic.operators.dataframes.ProjectRows;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionConstant;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionDataframe;

public class DFGraphTest {

	private final ProgramPoint fake;

	private final Variable df_foo, df_bar, df_foo_with_selection, df_bar_with_selection;

	private final ValueEnvironment<DFOrConstant> base;

	private final String fname_foo, fname_bar;

	private final ColumnListSelection selection;

	private final SelectionOperation<ColumnListSelection> selectionOp;

	private final ReadFromFile read_foo, read_bar;

//	private final PyClassType listtype;
	private final PyClassType slicetype;
	private final PyClassType dftype;
//	private final ReferenceType dfref;
	private final PyClassType seriestype;
//	private final ReferenceType seriesref;

	public DFGraphTest() throws SemanticException, AnalysisSetupException {
		Program p = new Program();
		LibrarySpecificationProvider.load(p);

//		listtype = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		slicetype = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
		dftype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
//		dfref = ((PyClassType) dftype).getReference();
		seriestype = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
//		seriesref = ((PyClassType) seriestype).getReference();

		ValueEnvironment<DFOrConstant> singleton = new ValueEnvironment<>(new DFOrConstant());

		fake = new ProgramPoint() {

			@Override
			public CodeLocation getLocation() {
				return SyntheticLocation.INSTANCE;
			}

			@Override
			public CFG getCFG() {
				return null;
			}
		};

		df_foo = new Variable(dftype, "df_foo", SyntheticLocation.INSTANCE);
		df_bar = new Variable(dftype, "df_bar", SyntheticLocation.INSTANCE);
		df_foo_with_selection = new Variable(dftype, "df_foo_with_selection",
				SyntheticLocation.INSTANCE);
		df_bar_with_selection = new Variable(dftype, "df_bar_with_selection",
				SyntheticLocation.INSTANCE);

		selection = new ColumnListSelection(new Names("foo"));
		selectionOp = new AccessOperation<>(fake.getLocation(), selection);

		fname_foo = "foo.csv";
		read_foo = new ReadFromFile(fake.getLocation(), fname_foo);
		DataframeGraphDomain elem = new DataframeGraphDomain(read_foo);
		DFOrConstant state = new DFOrConstant(elem);
		ValueEnvironment<DFOrConstant> env = singleton.putState(df_foo, state);
		elem = new DataframeGraphDomain(elem, selectionOp);
		state = new DFOrConstant(elem);
		env = env.putState(df_foo_with_selection, state);

		fname_bar = "bar.csv";
		read_bar = new ReadFromFile(fake.getLocation(), fname_bar);
		elem = new DataframeGraphDomain(read_bar);
		state = new DFOrConstant(elem);
		env = env.putState(df_bar, state);
		elem = new DataframeGraphDomain(elem, selectionOp);
		state = new DFOrConstant(elem);
		env = env.putState(df_bar_with_selection, state);

		base = env;
	}

	@Test
	public void allOperatorsAreTested() throws ClassNotFoundException {
		Reflections reflections = new Reflections(ReadDataframe.class.getPackageName(), new SubTypesScanner(false));
		Set<String> allTypes = reflections.getAllTypes();
		for (String type : allTypes) {
			Class<?> clazz = Class.forName(type);
			if (clazz.isInterface()
					|| Modifier.isAbstract(clazz.getModifiers())
					|| Modifier.isInterface(clazz.getModifiers()))
				continue;
			try {
				getClass().getDeclaredMethod("test" + type.substring(type.lastIndexOf('.') + 1));
			} catch (NoSuchMethodException e) {
				fail(type + " is not tested");
			}
		}
	}

	@Test
	public void testReadDataframe() throws SemanticException {
		Constant filename = new Constant(StringType.INSTANCE, fname_foo, SyntheticLocation.INSTANCE);
		UnaryExpression unary = new UnaryExpression(dftype, filename, ReadDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(new ReadFromFile(fake.getLocation(), fname_foo));

		assertEquals(expected, stack);
	}

	@Test
	public void testFilterNull() throws SemanticException {
		UnaryExpression unary = new UnaryExpression(dftype, df_foo, new FilterNull(Axis.ROWS),
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(),
				new FilterNullAxis(fake.getLocation(), Axis.ROWS));

		assertEquals(expected, stack);
	}

	@Test
	public void testAccessRows() throws SemanticException {
		TernaryExpression ternary = new TernaryExpression(dftype,
				df_foo,
				new Constant(Int32.INSTANCE, 0, SyntheticLocation.INSTANCE),
				new Constant(Int32.INSTANCE, 100, SyntheticLocation.INSTANCE),
				AccessRows.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(ternary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(),
				new AccessOperation<>(fake.getLocation(), new NumberSlice(0, 100)));

		assertEquals(expected, stack);
	}

	@Test
	public void testDropCols() throws SemanticException {
		BinaryExpression bin = new BinaryExpression(dftype,
				df_foo,
				new ListConstant(SyntheticLocation.INSTANCE,
						new DFOrConstant(new ConstantPropagation(
								new Constant(StringType.INSTANCE, "col1", SyntheticLocation.INSTANCE))),
						new DFOrConstant(new ConstantPropagation(
								new Constant(StringType.INSTANCE, "col2", SyntheticLocation.INSTANCE)))),
				DropCols.INSTANCE, SyntheticLocation.INSTANCE);

		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(bin, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();

		Set<String> colsShouldHaveAccessed = new HashSet<>();
		colsShouldHaveAccessed.add("col1");
		colsShouldHaveAccessed.add("col2");

		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(),
				new DropColumns(fake.getLocation(), new ColumnListSelection(new Names(colsShouldHaveAccessed))));

		assertEquals(expected, stack);
	}

	@Test
	public void testJoinCols() throws SemanticException {
		DataframeGraphDomain df2GraphDomain = new DataframeGraphDomain(
				base.getState(df_bar).df(),
				new DropColumns(fake.getLocation(), new ColumnListSelection(new Names("foo"))));

		ValueEnvironment<DFOrConstant> valEnv = base.putState(df_bar, new DFOrConstant(df2GraphDomain));
		BinaryExpression bin = new BinaryExpression(dftype, df_foo, df_bar, JoinCols.INSTANCE,
				SyntheticLocation.INSTANCE);

		ValueEnvironment<DFOrConstant> sss = valEnv.smallStepSemantics(bin, fake);

		DataframeGraph concatGraph = new DataframeGraph();
		concatGraph.mergeWith(df2GraphDomain.getTransformations());
		DataframeGraph left = base.getState(df_foo).df().getTransformations();
		concatGraph.mergeWith(left);

		DataframeOperation concat = new Concat(fake.getLocation(), Concat.Axis.CONCAT_COLS);

		concatGraph.addNode(concat);

		concatGraph.addEdge(new ConcatEdge(left.getLeaf(), concat, 0));
		concatGraph.addEdge(new ConcatEdge(df2GraphDomain.getTransformations().getLeaf(), concat, 1));

		DataframeGraphDomain expected = new DataframeGraphDomain(concatGraph);
		DataframeGraphDomain stack = sss.getValueOnStack().df();

		assertEquals(expected, stack);
	}

	@Test
	public void testAxisConcatenation() throws SemanticException {
		DataframeGraphDomain df2GraphDomain = new DataframeGraphDomain(
				base.getState(df_bar).df(),
				new DropColumns(fake.getLocation(), new ColumnListSelection(new Names("foo"))));

		ValueEnvironment<DFOrConstant> valEnv = base.putState(df_bar, new DFOrConstant(df2GraphDomain));
		UnaryExpression bin = new UnaryExpression(dftype,
				new ListConstant(fake.getLocation(), valEnv.getState(df_foo), valEnv.getState(df_bar)),
				new AxisConcatenation(Axis.ROWS),
				SyntheticLocation.INSTANCE);

		ValueEnvironment<DFOrConstant> sss = valEnv.smallStepSemantics(bin, fake);

		DataframeGraph concatGraph = new DataframeGraph();
		concatGraph.mergeWith(df2GraphDomain.getTransformations());
		DataframeGraph left = base.getState(df_foo).df().getTransformations();
		concatGraph.mergeWith(left);

		DataframeOperation concat = new Concat(fake.getLocation(), Concat.Axis.CONCAT_ROWS);

		concatGraph.addNode(concat);

		concatGraph.addEdge(new ConcatEdge(left.getLeaf(), concat, 0));
		concatGraph.addEdge(new ConcatEdge(df2GraphDomain.getTransformations().getLeaf(), concat, 1));

		DataframeGraphDomain expected = new DataframeGraphDomain(concatGraph);
		DataframeGraphDomain stack = sss.getValueOnStack().df();

		assertEquals(expected, stack);
	}

	@Test
	public void testProjectRows() throws SemanticException {
		TernaryExpression ternary = new TernaryExpression(dftype,
				df_foo,
				new Constant(Int32.INSTANCE, 0, SyntheticLocation.INSTANCE),
				new Constant(Int32.INSTANCE, 100, SyntheticLocation.INSTANCE),
				ProjectRows.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(ternary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(),
				new ProjectionOperation<>(fake.getLocation(), new NumberSlice(0, 100)));

		assertEquals(expected, stack);
	}

	@Test
	public void testApplyTransformation() throws SemanticException {
		UnaryExpression unary = new UnaryExpression(dftype,
				df_foo_with_selection,
				new ApplyTransformation(Kind.TO_DATETIME),
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(
				base.getState(df_foo_with_selection).df().getTransformations().prefix(),
				new Transform<>(fake.getLocation(), Kind.TO_DATETIME, selection));

		assertEquals(expected, stack);
	}

	public void testColumnAccess() throws SemanticException {
		BinaryExpression binary = new BinaryExpression(dftype,
				df_foo,
				new Constant(StringType.INSTANCE, "foo", fake.getLocation()),
				ColumnAccess.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(binary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(), selectionOp);

		assertEquals(expected, stack);
	}

	@Test
	public void testWriteSelectionDataframe() throws SemanticException {
		BinaryExpression binary = new BinaryExpression(dftype,
				df_foo_with_selection,
				df_bar,
				WriteSelectionDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(binary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();

		AssignDataframe<ColumnListSelection> assign = new AssignDataframe<>(fake.getLocation(), selection);

		DataframeGraph base = this.base.getState(df_foo_with_selection).df().getTransformations().prefix();
		base.addNode(assign);
		base.addNode(read_bar);
		base.addEdge(new SimpleEdge(read_foo, assign));
		base.addEdge(new AssignEdge(read_bar, assign));

		DataframeGraphDomain expected = new DataframeGraphDomain(base);

		assertEquals(expected, stack);
	}

	@Test
	public void testPopSelection() throws SemanticException {
		UnaryExpression unary = new UnaryExpression(dftype,
				df_foo_with_selection,
				PopSelection.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();

		DataframeGraphDomain expected = base.getState(df_foo).df();
		assertEquals(expected, stack);
	}

	@Test
	public void testPandasSeriesComparison() throws SemanticException {
		Set<String> cols = new HashSet<>();
		cols.add("col1");

		DataframeGraph initialGraph = new DataframeGraph();
		DataframeOperation read = new ReadFromFile(SyntheticLocation.INSTANCE, "foo1.csv");
		DataframeOperation projection = new ProjectionOperation<>(SyntheticLocation.INSTANCE,
				new ColumnListSelection(cols));
		initialGraph.addNode(read);
		initialGraph.addNode(projection);
		initialGraph.addEdge(new SimpleEdge(read, projection));
		ValueEnvironment<
				DFOrConstant> valEnv = base.putState(df_foo, new DFOrConstant(new DataframeGraphDomain(initialGraph)));

		BinaryExpression comparison = new BinaryExpression(seriestype, df_foo,
				new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE),
				new PandasSeriesComparison(ComparisonOperator.GEQ), SyntheticLocation.INSTANCE);
		valEnv = valEnv.smallStepSemantics(comparison, fake);

		DataframeGraph expectedGraph = initialGraph.prefix();
		AtomicBooleanSelection selection = new AtomicBooleanSelection(new ColumnListSelection(cols),
				ComparisonOperator.GEQ,
				new ConstantPropagation(new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE)));
		DataframeOperation boolComp = new BooleanComparison<>(SyntheticLocation.INSTANCE, selection);
		expectedGraph.addNode(boolComp);
		expectedGraph.addEdge(new SimpleEdge(read, boolComp));

		DataframeGraphDomain expected = new DataframeGraphDomain(expectedGraph);
		DataframeGraphDomain stack = valEnv.getValueOnStack().df();

		assertEquals(expected, stack);
	}

	@Test
	public void testAccessRowsColumns() throws SemanticException {
		// Part 1: access test with boolean condition on columns
		DataframeGraph df1Graph = new DataframeGraph();
		df1Graph = DataframeGraphDomain.append(df1Graph, new ReadFromFile(SyntheticLocation.INSTANCE, "foo.csv"));
		df1Graph = DataframeGraphDomain.append(df1Graph, new ProjectionOperation<ColumnListSelection>(
				SyntheticLocation.INSTANCE, new ColumnListSelection(new Names("col1"))));

		Variable df1 = new Variable(dftype, "df1", SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> env = base.putState(df1, new DFOrConstant(new DataframeGraphDomain(df1Graph)));

		ValueEnvironment<DFOrConstant> sss = env.smallStepSemantics(
				new BinaryExpression(seriestype, df1,
						new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE),
						new PandasSeriesComparison(ComparisonOperator.NEQ),
						SyntheticLocation.INSTANCE),
				fake);

		env = base.putState(df1, new DFOrConstant(new DataframeGraphDomain(df1Graph.prefix(),
				new AccessOperation<>(SyntheticLocation.INSTANCE, new ColumnListSelection(new Names("col1"))))));
		Variable df2 = new Variable(seriestype, "df2", SyntheticLocation.INSTANCE);
		env = env.putState(df2, sss.getValueOnStack());

		ListConstant cols = new ListConstant(SyntheticLocation.INSTANCE,
				new DFOrConstant(new ConstantPropagation(
						new Constant(StringType.INSTANCE, "col1", SyntheticLocation.INSTANCE))));
		
		sss = env.smallStepSemantics(
				new TernaryExpression(
						dftype, df1, df2,
						cols,
						AccessRowsColumns.INSTANCE, SyntheticLocation.INSTANCE),
				fake);

		ColumnListSelection expectedColumns = new ColumnListSelection(new Names("col1"));
		AtomicBooleanSelection expectedRows = new AtomicBooleanSelection(new ColumnListSelection(new Names("col1")),
				ComparisonOperator.NEQ,
				new ConstantPropagation(new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE)));
		DataframeSelection<AtomicBooleanSelection, ColumnListSelection> expectedSelection = new DataframeSelection<
				AtomicBooleanSelection, ColumnListSelection>(expectedRows, expectedColumns);
		DataframeGraph expected = DataframeGraphDomain.append(df1Graph.prefix(),
				new AccessOperation<>(SyntheticLocation.INSTANCE, expectedSelection));

		assertEquals(expected, sss.getValueOnStack().df().getTransformations());

		// Part 2: access test with row slice
		df1Graph = new DataframeGraph();
		df1Graph = DataframeGraphDomain.append(df1Graph, new ReadFromFile(SyntheticLocation.INSTANCE, "foo.csv"));

		env = base.putState(df1, new DFOrConstant(new DataframeGraphDomain(df1Graph)));

		sss = env.smallStepSemantics(
				new TernaryExpression(dftype, df1,
						new SliceConstant(new RangeBound(0),
								new RangeBound(2), null, SyntheticLocation.INSTANCE),
						cols,
						AccessRowsColumns.INSTANCE, SyntheticLocation.INSTANCE),
				fake);

		DataframeSelection<NumberSlice,
				ColumnListSelection> expectedSelection1 = new DataframeSelection<NumberSlice, ColumnListSelection>(
						new NumberSlice(0, 2), expectedColumns);
		expected = DataframeGraphDomain.append(df1Graph,
				new AccessOperation<>(SyntheticLocation.INSTANCE, expectedSelection1));

		assertEquals(expected, sss.getValueOnStack().df().getTransformations());
	}

	@Test
	public void testWriteSelectionConstant() throws SemanticException {
		// Part 1: write selection with a constant value
		Variable df1 = new Variable(dftype, "df1", SyntheticLocation.INSTANCE);
		DataframeGraph df1Graph = new DataframeGraph();
		df1Graph = DataframeGraphDomain.append(df1Graph, new ReadFromFile(SyntheticLocation.INSTANCE, "foo1.csv"));
		DataframeSelection<?, ?> selection = new DataframeSelection<>(new NumberSlice(25, 56),
				new ColumnListSelection(new Names("col1")));
		df1Graph = DataframeGraphDomain.append(df1Graph, new AccessOperation<>(SyntheticLocation.INSTANCE, selection));

		ValueEnvironment<DFOrConstant> env = base.putState(df1, new DFOrConstant(new DataframeGraphDomain(df1Graph)));

		ValueEnvironment<DFOrConstant> sss = env.smallStepSemantics(
				new BinaryExpression(dftype, df1,
						new Constant(StringType.INSTANCE, "hula-hoops", SyntheticLocation.INSTANCE),
						WriteSelectionConstant.INSTANCE, SyntheticLocation.INSTANCE),
				fake);

		DataframeGraph expected = DataframeGraphDomain.append(
				df1Graph.prefix(), new AssignValue<>(SyntheticLocation.INSTANCE, selection,
						new ConstantPropagation(
								new Constant(StringType.INSTANCE, "hula-hoops", SyntheticLocation.INSTANCE))));

		assertEquals(expected, sss.getValueOnStack().df().getTransformations());
	}

	@Test
	public void testSliceCreation() throws SemanticException {
		Variable start, skip;

		start = new Variable(Int32.INSTANCE, "start", SyntheticLocation.INSTANCE);
		skip = new Variable(Int32.INSTANCE, "skip", SyntheticLocation.INSTANCE);

		DFOrConstant startState = new DFOrConstant(
				new ConstantPropagation(new Constant(Int32.INSTANCE, 42, SyntheticLocation.INSTANCE)));
		DFOrConstant skipState = new DFOrConstant(
				new ConstantPropagation(new Constant(Int32.INSTANCE, 2, SyntheticLocation.INSTANCE)));
		ValueEnvironment<DFOrConstant> env = base.putState(start, startState);
		env = env.putState(skip, skipState);

		TernaryExpression slice1 = new TernaryExpression(
				slicetype,
				new Constant(NullType.INSTANCE, null, SyntheticLocation.INSTANCE),
				new Constant(NullType.INSTANCE, null, SyntheticLocation.INSTANCE),
				new Constant(NullType.INSTANCE, null, SyntheticLocation.INSTANCE),
				SliceCreation.INSTANCE,
				SyntheticLocation.INSTANCE);

		Constant slice1Constant = new SliceConstant(null, null, null, SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = env.smallStepSemantics(slice1, fake);
		assertEquals(new ConstantPropagation(slice1Constant), sss.getValueOnStack().constant());

		TernaryExpression slice2 = new TernaryExpression(
				slicetype,
				start,
				new Constant(NullType.INSTANCE, null, SyntheticLocation.INSTANCE),
				new Constant(NullType.INSTANCE, null, SyntheticLocation.INSTANCE),
				SliceCreation.INSTANCE,
				SyntheticLocation.INSTANCE);
		Constant slice2Constant = new SliceConstant(new RangeBound(42), null, null,
				SyntheticLocation.INSTANCE);
		sss = env.smallStepSemantics(slice2, fake);
		assertEquals(new ConstantPropagation(slice2Constant), sss.getValueOnStack().constant());

		TernaryExpression slice3 = new TernaryExpression(
				slicetype,
				start,
				new Constant(Int32.INSTANCE, 54, SyntheticLocation.INSTANCE),
				skip,
				SliceCreation.INSTANCE,
				SyntheticLocation.INSTANCE);
		Constant slice3Constant = new SliceConstant(new RangeBound(42), new RangeBound(54), new RangeBound(2),
				SyntheticLocation.INSTANCE);
		sss = env.smallStepSemantics(slice3, fake);
		assertEquals(new ConstantPropagation(slice3Constant), sss.getValueOnStack().constant());
	}
}
