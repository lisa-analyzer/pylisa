package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

import org.junit.Test;
import org.reflections.Reflections;
import org.reflections.scanners.SubTypesScanner;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
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
import it.unive.pylisa.analysis.dataframes.transformation.operations.BooleanComparison;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Concat;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DropColumns;
import it.unive.pylisa.analysis.dataframes.transformation.operations.FilterNullRows;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ProjectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.SelectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Transform;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.ColumnListSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.NumberSlice;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.symbolic.operators.AccessRows;
import it.unive.pylisa.symbolic.operators.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.ApplyTransformation.Kind;
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.ComparisonOperator;
import it.unive.pylisa.symbolic.operators.ConcatCols;
import it.unive.pylisa.symbolic.operators.ConcatRows;
import it.unive.pylisa.symbolic.operators.DropCols;
import it.unive.pylisa.symbolic.operators.FilterNull;
import it.unive.pylisa.symbolic.operators.PandasSeriesComparison;
import it.unive.pylisa.symbolic.operators.PopSelection;
import it.unive.pylisa.symbolic.operators.ProjectRows;
import it.unive.pylisa.symbolic.operators.ReadDataframe;
import it.unive.pylisa.symbolic.operators.WriteColumn;

public class DFGraphTest {

	private final ProgramPoint fake;

	private final Variable df_foo, df_bar, df_foo_with_selection, df_bar_with_selection;

	private final ValueEnvironment<DFOrConstant> base;

	private final String fname_foo, fname_bar;

	private final ColumnListSelection selection;

	private final SelectionOperation<ColumnListSelection> selectionOp;

	private final ReadFromFile read_foo, read_bar;

	public DFGraphTest() throws SemanticException {
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

		df_foo = new Variable(PandasDataframeType.INSTANCE, "df_foo", SyntheticLocation.INSTANCE);
		df_bar = new Variable(PandasDataframeType.INSTANCE, "df_bar", SyntheticLocation.INSTANCE);
		df_foo_with_selection = new Variable(PandasDataframeType.INSTANCE, "df_foo_with_selection",
				SyntheticLocation.INSTANCE);
		df_bar_with_selection = new Variable(PandasDataframeType.INSTANCE, "df_bar_with_selection",
				SyntheticLocation.INSTANCE);

		selection = new ColumnListSelection(new Names("foo"));
		selectionOp = new SelectionOperation<>(fake.getLocation(), selection);

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
		Reflections reflections = new Reflections("it.unive.pylisa.symbolic.operators", new SubTypesScanner(false));
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
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, filename, ReadDataframe.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(new ReadFromFile(fake.getLocation(), fname_foo));

		assertEquals(expected, stack);
	}

	@Test
	public void testFilterNull() throws SemanticException {
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE, df_foo, FilterNull.INSTANCE,
				SyntheticLocation.INSTANCE);
		ValueEnvironment<DFOrConstant> sss = base.smallStepSemantics(unary, fake);
		DataframeGraphDomain stack = sss.getValueOnStack().df();
		DataframeGraphDomain expected = new DataframeGraphDomain(base.getState(df_foo).df(),
				new FilterNullRows(fake.getLocation()));

		assertEquals(expected, stack);
	}

	@Test
	public void testAccessRows() throws SemanticException {
		TernaryExpression ternary = new TernaryExpression(PandasDataframeType.INSTANCE,
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

		@SuppressWarnings("unchecked")
		ExpressionSet<Constant>[] cols = (ExpressionSet<Constant>[]) new ExpressionSet[2];
		cols[0] = new ExpressionSet<>(new Constant(StringType.INSTANCE, "col1", SyntheticLocation.INSTANCE));
		cols[1] = new ExpressionSet<>(new Constant(StringType.INSTANCE, "col2", SyntheticLocation.INSTANCE));

		BinaryExpression bin = new BinaryExpression(PandasDataframeType.INSTANCE,
				df_foo,
				new Constant(PyListType.INSTANCE,
						cols,
						SyntheticLocation.INSTANCE),
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
	public void testConcatCols() throws SemanticException {
		DataframeGraphDomain df2GraphDomain = new DataframeGraphDomain(
				base.getState(df_bar).df(),
				new DropColumns(fake.getLocation(), new ColumnListSelection(new Names("foo"))));

		ValueEnvironment<DFOrConstant> valEnv = base.putState(df_bar, new DFOrConstant(df2GraphDomain));
		BinaryExpression bin = new BinaryExpression(PandasDataframeType.INSTANCE, df_foo, df_bar, ConcatCols.INSTANCE,
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
	public void testConcatRows() throws SemanticException {
		DataframeGraphDomain df2GraphDomain = new DataframeGraphDomain(
				base.getState(df_bar).df(),
				new DropColumns(fake.getLocation(), new ColumnListSelection(new Names("foo"))));

		ValueEnvironment<DFOrConstant> valEnv = base.putState(df_bar, new DFOrConstant(df2GraphDomain));
		BinaryExpression bin = new BinaryExpression(PandasDataframeType.INSTANCE, df_foo, df_bar, ConcatRows.INSTANCE,
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
		TernaryExpression ternary = new TernaryExpression(PandasDataframeType.INSTANCE,
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
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE,
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
		BinaryExpression binary = new BinaryExpression(PandasDataframeType.INSTANCE,
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
	public void testWriteColumn() throws SemanticException {
		BinaryExpression binary = new BinaryExpression(PandasDataframeType.INSTANCE,
				df_foo_with_selection,
				df_bar,
				WriteColumn.INSTANCE,
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
		UnaryExpression unary = new UnaryExpression(PandasDataframeType.INSTANCE,
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
		DataframeOperation projection = new ProjectionOperation<>(SyntheticLocation.INSTANCE, new ColumnListSelection(cols));
		initialGraph.addNode(read);
		initialGraph.addNode(projection);
		initialGraph.addEdge(new SimpleEdge(read, projection));
		ValueEnvironment<DFOrConstant> valEnv = base.putState(df_foo, new DFOrConstant(new DataframeGraphDomain(initialGraph)));

		BinaryExpression comparison = new BinaryExpression(PandasSeriesType.INSTANCE, df_foo, new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE), new PandasSeriesComparison(ComparisonOperator.GEQ), SyntheticLocation.INSTANCE);
		valEnv = valEnv.smallStepSemantics(comparison, fake);

		DataframeGraph expectedGraph = initialGraph.prefix();
		DataframeOperation boolComp = new BooleanComparison<>(SyntheticLocation.INSTANCE, new ColumnListSelection(cols), ComparisonOperator.GEQ, new ConstantPropagation(new Constant(Int32.INSTANCE, 5, SyntheticLocation.INSTANCE)));
		expectedGraph.addNode(boolComp);
		expectedGraph.addEdge(new SimpleEdge(read, boolComp));

		DataframeGraphDomain expected = new DataframeGraphDomain(expectedGraph);
		DataframeGraphDomain stack = valEnv.getValueOnStack().df();

		assertEquals(expected, stack);
	}
}
