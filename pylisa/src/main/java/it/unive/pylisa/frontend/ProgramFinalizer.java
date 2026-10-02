package it.unive.pylisa.frontend;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.statement.CarriedMarksStart;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.Objects;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Wraps the setup and finalisation work that sandwiches the visitor pass in
 * {@link PyFrontend#toLiSAProgram(boolean)}: registers primitive types, loads
 * the library specs + {@code builtins} before visiting, and — after visiting —
 * registers user-defined class types and synthesises the top-level
 * {@code $PythonProgram.run} CFG that acts as the LiSA entry point.
 * <p>
 * Extracted in Chunk 7 of the front-end refactor.
 */
public final class ProgramFinalizer {

	private static final Logger LOG = LogManager.getLogger(ProgramFinalizer.class);

	private static final SourceCodeLocation LISA_LOC = new SourceCodeLocation("__lisa__", 0, 0);

	private final ParserContext ctx;
	private final CFG init;

	public ProgramFinalizer(
			ParserContext ctx,
			CFG init) {
		this.ctx = Objects.requireNonNull(ctx);
		this.init = Objects.requireNonNull(init);
	}

	/**
	 * Pre-visit setup: registers built-in primitive types on the program's
	 * {@link TypeSystem}, loads the library specifications, imports
	 * {@code builtins}, and records the resolved {@code builtins.object} unit
	 * on the {@link ParserContext}. Must be called before the visitor runs.
	 */
	public void prepare() {
		registerPrimitiveTypes();
		loadLibrariesAndBuiltins();
	}

	/**
	 * Post-visit wrap-up: registers every {@link PyClassType} discovered during
	 * the visit on the program's type system, then synthesises the
	 * {@code $PythonProgram} unit and its {@code run} CFG (the LiSA entry
	 * point) that imports {@code builtins} followed by {@code __main__} and
	 * returns. Returns the fully-assembled program.
	 */
	public Program finalizeProgram() {
		registerUserTypes();
		return buildEntryPoint();
	}

	private void registerPrimitiveTypes() {
		TypeSystem types = ctx.program().getTypes();
		types.registerType(PyLambdaType.INSTANCE);
		types.registerType(BoolType.INSTANCE);
		types.registerType(StringType.INSTANCE);
		types.registerType(Int32Type.INSTANCE);
		types.registerType(Int64Type.INSTANCE);
		types.registerType(Float32Type.INSTANCE);
		types.registerType(Float64Type.INSTANCE);
		types.registerType(NullType.INSTANCE);
		types.registerType(VoidType.INSTANCE);
		types.registerType(Untyped.INSTANCE);
	}

	private void loadLibrariesAndBuiltins() {
		Program program = ctx.program();
		LibrarySpecificationProvider.load(program, init);
		LibrarySpecificationProvider.importPythonModule(program, "builtins", init);
		ctx.objectUnit((CompilationUnit) program.getUnit("builtins.object"));
		if (ctx.objectUnit() == null)
			LOG.warn("Could not resolve 'builtins.object' after library loading; "
					+ "classes without explicit parents will have no ancestor");
	}

	private void registerUserTypes() {
		TypeSystem types = ctx.program().getTypes();
		PyClassType.all().forEach(types::registerType);
	}

	private Program buildEntryPoint() {
		Program program = ctx.program();
		ModuleUnit pyProgramUnit = new ModuleUnit(LISA_LOC, program, "$PythonProgram");
		program.addUnit(pyProgramUnit);

		CodeMemberDescriptor runDesc = new CodeMemberDescriptor(LISA_LOC, pyProgramUnit, false, "run");
		runDesc.setOverridable(false);
		PyCFG runCFG = new PyCFG(runDesc);
		pyProgramUnit.addCodeMember(runCFG);

		// every carried mark starts unset, before any code runs
		CarriedMarksStart unset = new CarriedMarksStart(runCFG, LISA_LOC);
		runCFG.addNode(unset, true);
		ImportModule importBuiltins = new ImportModule(runCFG, LISA_LOC,
				"builtins", (ModuleUnit) PyModuleType.lookup("builtins").getUnit());
		runCFG.addNode(importBuiltins);
		runCFG.addEdge(new SequentialEdge(unset, importBuiltins));

		// Read the entry-file's compilation-unit name off the parser
		// context's currentModule rather than hardcoding "__main__".
		// PyFrontend optionally substitutes "__main__" with a dotted
		// module name (e.g. "mcpgateway.main") to match real Python
		// import-cache semantics under uvicorn-style deployments; this
		// synthetic import must use the same name or the PyModuleType
		// lookup below misses.
		ModuleUnit entryModule = (ModuleUnit) ctx.currentModule();
		String entryName = entryModule != null ? entryModule.getName() : "__main__";
		ImportModule importMain = new ImportModule(runCFG, LISA_LOC,
				entryName, (ModuleUnit) PyModuleType.lookup(entryName).getUnit());
		runCFG.addNode(importMain);
		runCFG.addEdge(new SequentialEdge(importBuiltins, importMain));

		Ret ret = new Ret(runCFG, PySyntheticLocation.INSTANCE);
		runCFG.addNode(ret);
		runCFG.addEdge(new SequentialEdge(importMain, ret));

		program.addEntryPoint(runCFG);
		return program;
	}
}
