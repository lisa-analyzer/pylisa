package it.unive.pylisa.frontend;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.statement.StatementVisitor;
import it.unive.pylisa.program.ModuleUnit;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;

/**
 * Installed on {@link PythonModuleImportManager} so recursive sub-module
 * imports (e.g. {@code from .config import X}) re-enter the front-end visitor
 * on the referenced file. Captures the current {@link ParserContext} state
 * before parsing the sub-module and restores it afterwards in a {@code
 * finally} block, so the entry file's visit resumes unchanged even if the
 * sub-module parse throws.
 * <p>
 * Extracted from {@code PyFrontend.loadProjectModuleFile} in Chunk 7; the
 * 7-pair save/restore is collapsed into the {@link Snapshot} record.
 */
public final class ModuleLoaderCallback {

	private final ParserContext ctx;
	private final ParserSupport support;
	private final StatementVisitor stmt;

	public ModuleLoaderCallback(
			ParserContext ctx,
			ParserSupport support,
			StatementVisitor stmt) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
		this.stmt = Objects.requireNonNull(stmt);
	}

	/**
	 * Parses the given project file as a sub-module and returns its
	 * {@link ModuleUnit}. Uses a fresh {@link AntlrPipeline} so the recursive
	 * parse sees a token stream whose source name matches the sub-module path
	 * (required for correct {@link it.unive.lisa.program.SourceCodeLocation}
	 * stamping on nested visits).
	 *
	 * @param moduleName the dotted name already registered via
	 *                       {@link PyModuleType#register(String, ModuleUnit)}
	 * @param modulePath the filesystem path of the sub-module
	 *
	 * @return the {@link ModuleUnit} for the loaded sub-module
	 *
	 * @throws IOException if the file cannot be read or parsed
	 */
	public ModuleUnit load(
			String moduleName,
			String modulePath)
			throws IOException {
		Snapshot snap = Snapshot.capture(ctx);
		ctx.currentFileIsPackage(modulePath.endsWith("__init__.py"));

		// The ModuleUnit was already registered by loadProjectModule before
		// calling us.
		ModuleUnit newModule = (ModuleUnit) PyModuleType.lookup(moduleName).getUnit();
		ctx.currentModule(newModule);
		ctx.currentUnit(newModule);
		ctx.imports(new HashMap<>());
		ctx.cfs(new HashSet<>());
		ctx.continueOnUnsupportedStatement(true);

		try {
			String source = SourceReader.readNormalizedFile(modulePath);
			stmt.visitFile_input(new AntlrPipeline(modulePath, source).parseFile());
		} catch (IOException | RuntimeException e) {
			// Finalize the partially-built $init CFG so it passes LiSA
			// validation even when the sub-module parse aborts.
			support.addRetNodesToCurrentCFG();
			throw e;
		} finally {
			snap.restore(ctx);
		}

		return newModule;
	}

	/**
	 * Immutable snapshot of the subset of {@link ParserContext} fields that
	 * {@link #load} overwrites, captured on entry and restored on exit. Adding
	 * a new field here is the only place to remember to protect it across
	 * recursive sub-module parses.
	 */
	private record Snapshot(
			ModuleUnit module,
			Unit unit,
			PyCFG cfg,
			Map<String, String> imports,
			Collection<ControlFlowStructure> cfs,
			boolean prependUnitAccess,
			boolean fileIsPackage,
			boolean continueOnUnsupported) {

		static Snapshot capture(
				ParserContext ctx) {
			return new Snapshot(
					ctx.currentModule(),
					ctx.currentUnit(),
					ctx.currentCFG(),
					ctx.imports(),
					ctx.cfs(),
					ctx.shouldPrependUnitAccess(),
					ctx.currentFileIsPackage(),
					ctx.continueOnUnsupportedStatement());
		}

		void restore(
				ParserContext ctx) {
			ctx.continueOnUnsupportedStatement(continueOnUnsupported);
			ctx.currentModule(module);
			ctx.currentUnit(unit);
			ctx.currentCFG(cfg);
			ctx.imports(imports);
			ctx.cfs(cfs);
			ctx.shouldPrependUnitAccess(prependUnitAccess);
			ctx.currentFileIsPackage(fileIsPackage);
		}
	}
}
