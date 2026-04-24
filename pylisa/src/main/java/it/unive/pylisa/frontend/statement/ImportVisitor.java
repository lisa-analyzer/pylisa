package it.unive.pylisa.frontend.statement;

import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Import_fromContext;
import it.unive.pylisa.antlr.Python3Parser.Import_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_stmtContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.frontend.expression.DunderMethods;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Handles Python import statements and the {@code dotted_name} production.
 * Extracted from {@link StatementVisitor} in Chunk 4.
 */
public final class ImportVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public ImportVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Statement visitImport_stmt(
			Import_stmtContext pctx) {
		if (pctx.import_from() != null)
			return visitImport_from(pctx.import_from());
		else
			return visitImport_name(pctx.import_name());
	}

	public Statement visitImport_name(
			Import_nameContext pctx) {
		Map<String, String> libs = new HashMap<>();
		for (Dotted_as_nameContext single : pctx.dotted_as_names().dotted_as_name()) {
			String importedLibrary = dottedNameToString(single.dotted_name());
			String as = single.NAME() != null ? single.NAME().getSymbol().getText() : null;
			libs.put(importedLibrary, as);
		}

		Map.Entry<String, String> binding = libs.entrySet().stream().findFirst().orElse(null);
		if (binding != null) {
			String boundName = binding.getValue() == null ? binding.getKey() : binding.getValue();
			ctx.declareNameInCurrentScope(boundName);
		}

		String moduleName = libs.keySet().stream().findFirst().get();
		ModuleUnit module = ctx.importManager().importModule(moduleName);
		return new ImportModule(ctx.currentCFG(), support.getLocation(pctx), moduleName, module);
	}

	public Statement visitImport_from(
			Import_fromContext pctx) {
		int dotCount = pctx.DOT().size() + pctx.ELLIPSIS().size() * 3;
		String name;
		if (dotCount == 0) {
			name = pctx.dotted_name() != null ? dottedNameToString(pctx.dotted_name()) : ".";
		} else {
			name = resolveRelativeImport(dotCount,
					pctx.dotted_name() != null ? dottedNameToString(pctx.dotted_name()) : null);
		}

		if (pctx.import_as_names() == null) {
			LibrarySpecificationProvider.importLibrary(ctx.program(), name, ctx.init());
			PyCFG pyCFG = new PyCFG(
					new CodeMemberDescriptor(support.getLocation(pctx), ctx.currentUnit(), false, DunderMethods.INIT));
			pyCFG.addNode(new NoOp(pyCFG, SyntheticLocation.INSTANCE));
			return new ImportModule(ctx.currentCFG(), support.getLocation(pctx), name,
					(ModuleUnit) PyModuleType.lookup(name).getUnit());
		}
		Map<String, String> components = new LinkedHashMap<>();
		for (Import_as_nameContext single : pctx.import_as_names().import_as_name()) {
			String importedComponent = single.NAME(0).getSymbol().getText();
			String alias = single.NAME().size() == 2 ? single.NAME(1).getSymbol().getText() : importedComponent;
			components.put(importedComponent, alias);
			ctx.imports().put(alias, name + "." + importedComponent);
			ctx.declareNameInCurrentScope(alias);
		}
		ctx.importManager().importModule(name);

		for (Map.Entry<String, String> entry : components.entrySet()) {
			String qualifiedName = name + "." + entry.getKey();
			if (ctx.importManager().canResolveModule(qualifiedName))
				ctx.importManager().importModule(qualifiedName);
		}

		ModuleUnit currentModuleUnit = (ctx.currentUnit() instanceof ModuleUnit pmu) ? pmu : null;
		List<Pair<String, String>> importPairs = components.entrySet().stream()
				.map(e -> Pair.of(e.getValue(), e.getKey()))
				.collect(Collectors.toList());
		return new FromImport(ctx.currentCFG(), support.getLocation(pctx), currentModuleUnit, name, importPairs);
	}

	public Object visitImport_as_name(
			Import_as_nameContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitDotted_as_name(
			Dotted_as_nameContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitImport_as_names(
			Import_as_namesContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitDotted_as_names(
			Dotted_as_namesContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Expression visitDotted_name(
			Dotted_nameContext pctx) {
		if (pctx.NAME().isEmpty()) {
			return support.unsupported(pctx, "At least one name expected in Dotted_nameContext");
		}
		Expression result = null;
		String targetName = null;
		for (int i = 0; i < pctx.NAME().size(); i++) {
			TerminalNode name = pctx.NAME(i);
			if (targetName != null) {
				result = new AttributeAccess(ctx.currentCFG(), support.getLocation(pctx), result, name.getText());
			} else {
				result = support.makeRef(name.getText(), support.getLocation(pctx));
			}
			targetName = name.getText();
		}
		return result;
	}

	public String dottedNameToString(
			Dotted_nameContext dotted_name) {
		return dotted_name.NAME().stream().map(t -> t.getSymbol().getText())
				.collect(Collectors.joining("."));
	}

	public String resolveRelativeImport(
			int dotCount,
			String dottedName) {
		String moduleName = (ctx.currentModule() != null) ? ctx.currentModule().getName() : "__main__";
		String packageName;
		if (ctx.currentFileIsPackage()) {
			packageName = moduleName;
		} else {
			int lastDot = moduleName.lastIndexOf('.');
			packageName = (lastDot >= 0) ? moduleName.substring(0, lastDot) : "";
		}
		// Entry-file fallback: the top-level main file carries the hardcoded
		// module name "__main__" and therefore has no package prefix to pull
		// from. When it lives inside a package (baseDir has __init__.py), the
		// import manager has already detected the dotted package name from the
		// directory chain — use it as the anchor, matching Python's
		// __package__ behavior for `python -m <pkg>.<mod>` execution.
		if (packageName.isEmpty()) {
			String detected = ctx.importManager().getPackageName();
			if (detected != null)
				packageName = detected;
		}
		String anchor = packageName;
		for (int i = 1; i < dotCount; i++) {
			int lastDot = anchor.lastIndexOf('.');
			anchor = (lastDot >= 0) ? anchor.substring(0, lastDot) : "";
		}
		if (dottedName == null || dottedName.isEmpty())
			return anchor;
		return anchor.isEmpty() ? dottedName : anchor + "." + dottedName;
	}
}
