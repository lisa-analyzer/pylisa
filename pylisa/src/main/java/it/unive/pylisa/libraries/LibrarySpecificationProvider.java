package it.unive.pylisa.libraries;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.pylisa.antlr.LibraryDefinitionLexer;
import it.unive.pylisa.antlr.LibraryDefinitionParser;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;

public class LibrarySpecificationProvider {

	public static final String SET = "Set";
	public static final String DICT = "Dict";
	public static final String LIST = "List";
	public static final String TUPLE = "Tuple";
	public static final String SLICE = "Slice";
	public static final String OBJECT = "Object";

	public static final String WARNINGS = "warnings";

	public static final String PANDAS = "pandas";
	public static final String PANDAS_DF = "pandas.DataFrame";
	public static final String PANDAS_SERIES = "pandas.Series";

	public static final String GEOPANDAS = "geopandas";
	public static final String GEOPANDAS_TOOLS = "geopandas.tools";

	public static final String NUMPY_ARRAY = "numpy.NDArray";

	private static final Map<String, CompilationUnit> LIBS = new HashMap<>();

	public static CompilationUnit hierarchyRoot;

	public static void load(Program program) throws AnalysisSetupException {
		String file = "/libs.txt";

		LibraryDefinitionLexer lexer = null;
		try (InputStream stream = LibrarySpecificationParser.class.getResourceAsStream(file);) {
			lexer = new LibraryDefinitionLexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new AnalysisSetupException("Unable to parse '" + file + "'", e);
		}

		LibraryDefinitionParser parser = new LibraryDefinitionParser(new CommonTokenStream(lexer));
		LibrarySpecificationParser libParser = new LibrarySpecificationParser(file, program);
		Collection<CompilationUnit> parsed = libParser.visitFile(parser.file());
		hierarchyRoot = libParser.root;

		for (CompilationUnit unit : parsed)
			LIBS.put(unit.getName(), unit);
	}

	public static Collection<CompilationUnit> getLibraryUnits() {
		return LIBS.values();
	}

	public static CompilationUnit getLibraryUnit(String name) {
		return LIBS.get(name);
	}
}
