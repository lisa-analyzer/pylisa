package it.unive.pylisa;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;
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
		int n = expandedPaths.size();

		// Parse all files once upfront
		for (int i = 0; i < n; i++) {
			Path path = Paths.get(expandedPaths.get(i));
			String source = path.toString();
			PyFileParser pyFileParser = new PyFileParser(program, source, source.endsWith(".ipynb"), cellOrder);
			pyFileParser.parse();
		}

		registerTypes();

		for (CFG cm : program.getAllCFGs())
			if (cm.getDescriptor().getName().equals(PyFileParser.INSTRUMENTED_MAIN_FUNCTION_NAME))
				program.addEntryPoint(cm);

		return program;
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

}
