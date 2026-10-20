package it.unive.pylisa;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.pylisa.antlr.PythonLexer;
import it.unive.pylisa.antlr.PythonParser;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.stream.Stream;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class PyFrontend
		extends
		PythonParserBaseVisitor<Object> {

	private static final Logger log = LogManager.getLogger(PyFrontend.class);

	/**
	 * The LiSA program obtained from the Python program at filePath.
	 */
	private final Program program;

	public PyFrontend() {
		clearAll();
		this.program = new Program(new PythonFeatures(), new PythonTypeSystem());
	}

	public void clearAll() {
		PyClassType.clearAll();
	}

	public Program getProgram() {
		return program;
	}

	private void registerTypes() {
		TypeSystem types = program.getTypes();
		types.registerType(PyLambdaType.INSTANCE);
		types.registerType(BoolType.INSTANCE);
		types.registerType(StringType.INSTANCE);
		types.registerType(PyBytesType.INSTANCE);
		types.registerType(Int32Type.INSTANCE);
		types.registerType(Float32Type.INSTANCE);
		types.registerType(NullType.INSTANCE);
		types.registerType(VoidType.INSTANCE);
		types.registerType(Untyped.INSTANCE);

		PyClassType.all().forEach(types::registerType);
	}

	public Program parseFromListOfFile(
			List<String> filePaths)
			throws IOException {
		return parseFromListOfFile(filePaths, Collections.emptyList());
	}

	public Program parseFromListOfFile(
			List<String> filePaths,
			List<Integer> cellOrder)
			throws IOException {
		LibrarySpecificationProvider.load(program);
		LibrarySpecificationProvider.importBuiltins(program);
		List<String> expandedPaths = expandFilePaths(filePaths);
		// the local modules are looked up in the directory of the program
		Path root = expandedPaths.isEmpty() ? Paths.get("") : directory(Paths.get(expandedPaths.get(0)));
		Set<Path> parsed = new HashSet<>();
		for (String path : expandedPaths)
			parse(Paths.get(path), root, cellOrder, parsed);

		registerTypes();

		for (CFG cm : program.getAllCFGs())
			checkDistinctNodes(cm);

		for (CFG cm : program.getAllCFGs())
			if (cm.getDescriptor().getName().equals(PyFileParser.INSTRUMENTED_MAIN_FUNCTION_NAME))
				program.addEntryPoint(cm);

		return program;
	}

	private static String transformToCode(
			List<String> code_list) {
		StringBuilder result = new StringBuilder();
		for (String s : code_list)
			result.append(s).append("\n");
		return result.toString();
	}

	private InputStream mkStream(
			String filePath,
			List<Integer> cellOrder)
			throws FileNotFoundException {
		if (!filePath.endsWith(".ipynb"))
			return new FileInputStream(filePath);

		Gson gson = new Gson();
		JsonReader reader = gson.newJsonReader(new FileReader(filePath));
		Map<?, ?> map = gson.fromJson(reader, Map.class);
		List<Map<?, ?>> cells = (ArrayList<Map<?, ?>>) map.get("cells");
		SortedMap<Integer, String> codeBlocks = new TreeMap<>();
		for (int i = 0; i < cells.size(); i++) {
			Map<?, ?> cell = cells.get(i);
			String ctype = (String) cell.get("cell_type");
			if (ctype.equals("code")) {
				List<String> code_list = (List<String>) cell.get("source");
				codeBlocks.put(i, transformToCode(code_list));
			}
		}

		StringBuilder code = new StringBuilder();

		if (cellOrder.isEmpty())
			for (Entry<Integer, String> c : codeBlocks.entrySet())
				code.append(c.getValue()).append("\n");
		else {
			log.warn("The following cells contain code and can be analyzed: " + codeBlocks.keySet());
			for (int idx : cellOrder) {
				String str = codeBlocks.get(idx);
				if (str == null)
					log.warn("Cell " + idx + " does not contain code and will be skipped");
				else
					code.append(str).append("\n");
			}
		}

		return new ByteArrayInputStream(code.toString().getBytes());
	}

	protected List<String> expandFilePaths(
			List<String> paths)
			throws IOException {
		java.util.List<String> expandedPaths = new java.util.ArrayList<>();
		for (String pathStr : paths) {
			Path path = Paths.get(pathStr).normalize();
			if (Files.isDirectory(path)) {
				try (Stream<Path> stream = Files.walk(path)) {
					stream.filter(Files::isRegularFile)
							.filter(p -> p.toString().endsWith(".py") || p.toString().endsWith(".ipynb"))
							.forEach(p -> expandedPaths.add(p.toString()));
				}
			} else if (Files.isRegularFile(path)) {
				if (path.toString().endsWith(".py") || path.toString().endsWith(".ipynb"))
					expandedPaths.add(path.toString());
				else
					log.warn("File {} is not a Python source file (.py or .ipynb), skipping.", pathStr);
			} else {
				throw new FileNotFoundException(pathStr);
			}
		}
		return expandedPaths;
	}

	// parses the file, after the local modules it imports (each file is
	// parsed once)
	private void parse(
			Path path,
			Path root,
			List<Integer> cellOrder,
			Set<Path> parsed)
			throws IOException {
		if (!parsed.add(path.normalize()))
			return;
		String source = path.toString();
		log.info("Reading file... " + source);

		PythonLexer lexer = null;
		try (InputStream stream = mkStream(source, cellOrder);) {
			lexer = new PythonLexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + source + "'", e);
		}

		PythonParser parser = new PythonParser(new CommonTokenStream(lexer));
		File_inputContext tree = parser.file_input();

		// the modules are parsed first, so that their functions and classes
		// are known when parsing this file
		LocalModules.Imports imports = LocalModules.of(tree, path, root);
		for (Path module : imports.dependencies())
			parse(module, root, Collections.emptyList(), parsed);

		PyFileParser pyFileParser = new PyFileParser(program, source, imports.bindings());
		program.addUnit(pyFileParser.parse(tree));
	}

	// the directory containing the file (the current one if it has no parent)
	private static Path directory(
			Path file) {
		Path parent = file.getParent();
		return parent == null ? Paths.get("") : parent;
	}

	/**
	 * Checks that the nodes of the cfg (statements and expressions) are
	 * distinct: each one has a single parent, and no two of them are equal. The
	 * results of the analysis are stored for each node, and equal nodes would
	 * share them: which one is kept would depend on the order of the
	 * computation, making the results not deterministic.
	 *
	 * @param cfg the cfg
	 *
	 * @throws IllegalStateException if two nodes are not distinct
	 */
	private static void checkDistinctNodes(
			CFG cfg) {
		Map<Statement, Boolean> seen = new IdentityHashMap<>();
		Map<Statement, Statement> equal = new HashMap<>();
		Deque<Statement> todo = new ArrayDeque<>(cfg.getNodes());
		while (!todo.isEmpty()) {
			Statement st = todo.pop();
			if (seen.put(st, true) != null)
				throw new IllegalStateException("The node " + st + " at " + st.getLocation() + " of "
						+ cfg.getDescriptor().getName() + " has more than one parent");
			Statement other = equal.putIfAbsent(st, st);
			if (other != null)
				throw new IllegalStateException("The nodes " + st + " and " + other + " at " + st.getLocation()
						+ " of " + cfg.getDescriptor().getName() + " are equal");
			Expression[] subs = st instanceof NaryExpression ? ((NaryExpression) st).getSubExpressions()
					: st instanceof NaryStatement ? ((NaryStatement) st).getSubExpressions() : new Expression[0];
			for (Expression sub : subs)
				todo.push(sub);
		}
	}
}
