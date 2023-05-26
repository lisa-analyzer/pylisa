package it.unive.pylisa.libraries;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.apache.commons.lang3.tuple.Pair;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.pylisa.antlr.LibraryDefinitionLexer;
import it.unive.pylisa.antlr.LibraryDefinitionParser;
import it.unive.pylisa.libraries.loader.Library;
import it.unive.pylisa.libraries.loader.Runtime;

public class LibrarySpecificationProvider {

	private static final String LIBS_FILE = "/libs.txt";
	
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

	private static final Map<String, Library> AVAILABLE_LIBS = new HashMap<>();

	public static CompilationUnit hierarchyRoot;

	private static CFG init;
	
	private static final Collection<String> LOADED_LIBS = new HashSet<>();

	public static void load(Program program) throws AnalysisSetupException {
		init = null;
		hierarchyRoot = null;
		AVAILABLE_LIBS.clear();
		LOADED_LIBS.clear();

		LibraryDefinitionLexer lexer = null;
		try (InputStream stream = LibrarySpecificationParser.class.getResourceAsStream(LIBS_FILE);) {
			lexer = new LibraryDefinitionLexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new AnalysisSetupException("Unable to parse '" + LIBS_FILE + "'", e);
		}

		LibraryDefinitionParser parser = new LibraryDefinitionParser(new CommonTokenStream(lexer));
		LibrarySpecificationParser libParser = new LibrarySpecificationParser(LIBS_FILE);
		Pair<Runtime, Collection<Library>> parsed = libParser.visitFile(parser.file());

		AtomicReference<CompilationUnit> root = new AtomicReference<CompilationUnit>(null);
		parsed.getLeft().fillProgram(program, root);
		if (root.get() == null)
			throw new AnalysisSetupException("Runtime does not contain a hierarchy root");
		hierarchyRoot = root.get();
		makeInit(program);
		parsed.getLeft().populateProgram(program, init, hierarchyRoot);

		for (Library lib : parsed.getValue())
			AVAILABLE_LIBS.put(lib.getName(), lib);
	}

	private static CFG makeInit(Program program) {
		init = new CFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, SyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}

	public static void importLibrary(Program program, String name) {
		if (LOADED_LIBS.contains(name))
			return;
		
		Library library = AVAILABLE_LIBS.get(name);
		if (library == null)
			// TODO do we log? could also be imports to other files under analysis...
			return;
		CodeUnit lib = library.toLiSAUnit(program, new AtomicReference<>(hierarchyRoot));
		library.populateUnit(init, hierarchyRoot, lib);
		LOADED_LIBS.add(name);
	}

	public static Collection<Library> getLibraryUnits() {
		return AVAILABLE_LIBS.values();
	}

	public static Library getLibraryUnit(String name) {
		return AVAILABLE_LIBS.get(name);
	}
	
	public static boolean isLibraryLoaded(String name) {
		return LOADED_LIBS.contains(name);
	}
}
