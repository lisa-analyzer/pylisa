package it.unive.pylisa.libraries;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.type.common.Int32;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.cfg.PythonUnit;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.libraries.geopandas.Geocode;
import it.unive.pylisa.libraries.pandas.Apply;
import it.unive.pylisa.libraries.pandas.Concatenate;
import it.unive.pylisa.libraries.pandas.DropNA;
import it.unive.pylisa.libraries.pandas.Head;
import it.unive.pylisa.libraries.pandas.Join;
import it.unive.pylisa.libraries.pandas.ReadCsv;
import it.unive.pylisa.libraries.pandas.ToDatetime;
import it.unive.pylisa.libraries.pandas.types.PandasDataframeType;
import it.unive.pylisa.libraries.pandas.types.PandasSeriesType;
import it.unive.pylisa.libraries.standardLibrary.Print;
import it.unive.pylisa.libraries.warnings.FilterWarnings;

public class LibrarySpecificationProvider {
	public static final String SET = "set";
	public static final String DICT = "dict";
	public static final String LIST = "list";
	public static final String TUPLE = "tuple";
	public static final String WARNINGS = "warnings";
	public static final String SLICE = "slice";
	public static final String PANDAS = "pandas";
	public static final String PANDAS_DF = "pandas.DataFrame";
	public static final String PANDAS_SERIES = "pandas.Series";
	public static final String GEOPANDAS = "geopandas";
	public static final String GEOPANDAS_TOOLS = "geopandas.tools";

	private static final SourceCodeLocation PANDAS_LOC = new SourceCodeLocation(PANDAS, 0, 0);
	private static final SourceCodeLocation GEOPANDAS_LOC = new SourceCodeLocation(GEOPANDAS, 0, 0);
	private static final SourceCodeLocation WARNINGS_LOC = new SourceCodeLocation(WARNINGS, 0, 0);
	private static final SourceCodeLocation STDLIB_LOC = new SourceCodeLocation("standard_library", 0, 0);

	private static final Map<String, CompilationUnit> LIBS = new HashMap<>();

	static {
		LIBS.put(PANDAS, getPandasPythonUnit());
		LIBS.put(PANDAS_DF, getPandasDFPythonUnit());
		LIBS.put(PANDAS_SERIES, getPandasSeriesPythonUnit());
		LIBS.put(WARNINGS, getWarningsPythonUnit());
		LIBS.put(LIST, getListPythonUnit());
		LIBS.put(DICT, getDictPythonUnit());
		LIBS.put(SET, getSetPythonUnit());
		LIBS.put(TUPLE, getTuplePythonUnit());
		LIBS.put(SLICE, getSlicePythonUnit());
		LIBS.put(GEOPANDAS_TOOLS, getGeopandasToolsPythonUnit());
	}

	public static Collection<CompilationUnit> getLibraryUnits() {
		return LIBS.values();
	}

	public static CompilationUnit getLibraryUnit(String name) {
		return LIBS.get(name);
	}

	public static Collection<NativeCFG> getAllStandardLibraryMethods(Program program) {
		Set<NativeCFG> result = new HashSet<>();
		result.add(new NativeCFG(
				new CFGDescriptor(STDLIB_LOC,
						program,
						false,
						"print",
						new Parameter(STDLIB_LOC, "arg1")),
				Print.class));
		return result;
	}

	private static PythonUnit getListPythonUnit() {
		PythonUnit unit1 = new PythonUnit(STDLIB_LOC, "List", true);
		return unit1;
	}

	private static PythonUnit getDictPythonUnit() {
		PythonUnit unit1 = new PythonUnit(STDLIB_LOC, "Dict", true);
		return unit1;
	}

	private static PythonUnit getSetPythonUnit() {
		PythonUnit unit1 = new PythonUnit(STDLIB_LOC, "Set", true);
		return unit1;
	}

	private static PythonUnit getTuplePythonUnit() {
		PythonUnit unit1 = new PythonUnit(STDLIB_LOC, "Tuple", true);
		return unit1;
	}

	private static PythonUnit getWarningsPythonUnit() {
		PythonUnit unit1 = new PythonUnit(WARNINGS_LOC, WARNINGS, true);
		NativeCFG cfg = new NativeCFG(
				new CFGDescriptor(WARNINGS_LOC,
						unit1,
						false,
						"filterwarnings",
						new Parameter(WARNINGS_LOC, "arg1")),
				FilterWarnings.class);
		unit1.addInstanceConstruct(cfg);
		return unit1;
	}

	private static PythonUnit getSlicePythonUnit() {
		PythonUnit unit1 = new PythonUnit(STDLIB_LOC, "slice", true);
		return unit1;
	}

	private static PythonUnit getPandasPythonUnit() {
		PythonUnit unit1 = new PythonUnit(PANDAS_LOC, PANDAS, true);
		unit1.addConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						false,
						"read_csv",
						PandasDataframeType.REFERENCE,
						new Parameter(PANDAS_LOC, "filepath_or_buffer", StringType.INSTANCE)),
				ReadCsv.class));

		unit1.addConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						false,
						"to_datetime",
						PandasSeriesType.REFERENCE,
						new Parameter(PANDAS_LOC, "arg", PandasSeriesType.REFERENCE)),
				ToDatetime.class));

		unit1.addConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						false,
						"concat",
						PandasDataframeType.REFERENCE,
						new Parameter(PANDAS_LOC, "objs", PyListType.INSTANCE)),
				Concatenate.class));

		return unit1;
	}

	private static PythonUnit getPandasDFPythonUnit() {
		PythonUnit unit1 = new PythonUnit(PANDAS_LOC, PANDAS_DF, true);

		// cfg for plugging default values evaluation
		CFG init = new CFG(new CFGDescriptor(PANDAS_LOC, unit1, false, "init"));
		init.addNode(new Ret(init, PANDAS_LOC), true);
		unit1.addCFG(init);

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"head",
						PandasDataframeType.REFERENCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE),
						new Parameter(PANDAS_LOC, "n", Int32.INSTANCE, new Int32Literal(init, PANDAS_LOC, 5),
								new Annotations())),
				Head.class));

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"describe",
						PandasDataframeType.REFERENCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE)),
				UninterestingDataframeFunction.class));

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"info",
						VoidType.INSTANCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE)),
				UninterestingDataframeFunction.class));

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"dropna",
						Untyped.INSTANCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE),
						new Parameter(PANDAS_LOC, "axis", Int32.INSTANCE, new Int32Literal(init, PANDAS_LOC, 0),
								new Annotations()),
						new Parameter(PANDAS_LOC, "inplace", BoolType.INSTANCE, new FalseLiteral(init, PANDAS_LOC),
								new Annotations())),
				DropNA.class));

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"apply",
						Untyped.INSTANCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE),
						new Parameter(PANDAS_LOC, "function", PyLambdaType.INSTANCE)),
				Apply.class));

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"join",
						PandasDataframeType.REFERENCE,
						new Parameter(PANDAS_LOC, "this", PandasDataframeType.REFERENCE),
						new Parameter(PANDAS_LOC, "other", PandasDataframeType.REFERENCE)),
				Join.class));

		return unit1;
	}

	private static PythonUnit getPandasSeriesPythonUnit() {
		PythonUnit unit1 = new PythonUnit(PANDAS_LOC, PANDAS_SERIES, true);

		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(PANDAS_LOC,
						unit1,
						true,
						"apply",
						Untyped.INSTANCE,
						new Parameter(PANDAS_LOC, "this", PandasSeriesType.REFERENCE),
						new Parameter(PANDAS_LOC, "function", PyLambdaType.INSTANCE)),
				Apply.class));

		return unit1;
	}

	private static PythonUnit getGeopandasToolsPythonUnit() {
		PythonUnit unit1 = new PythonUnit(GEOPANDAS_LOC, GEOPANDAS_TOOLS, true);

		// cfg for plugging default values evaluation
		CFG init = new CFG(new CFGDescriptor(GEOPANDAS_LOC, unit1, false, "init"));
		init.addNode(new Ret(init, GEOPANDAS_LOC), true);
		unit1.addCFG(init);

		unit1.addConstruct(new NativeCFG(
				new CFGDescriptor(GEOPANDAS_LOC,
						unit1,
						false,
						"geocode",
						PandasDataframeType.REFERENCE,
						new Parameter(GEOPANDAS_LOC, "strings", PandasSeriesType.REFERENCE),
						new Parameter(GEOPANDAS_LOC, "provider", StringType.INSTANCE,
								new StringLiteral(init, GEOPANDAS_LOC, "default"), new Annotations())),
				Geocode.class));

		return unit1;
	}
}
