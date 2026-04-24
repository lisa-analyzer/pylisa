package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.ClassdefContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.ImportClass;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python class definitions: {@code classdef} plus the body walk
 * ({@code parseClassBody} / {@code parseField}). Extracted from
 * {@link DefinitionVisitor} in Chunk 5.
 */
public final class ClassDefinitionVisitor {

	private static final Logger LOG = LogManager.getLogger(ClassDefinitionVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public ClassDefinitionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitClassdef(
			ClassdefContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Unit previous = ctx.currentUnit();
		String name = pctx.NAME().getSymbol().getText();
		String baseFqName = previous.getName() + "." + name;
		// Allocation-site abstraction: each `class` statement mints a fresh
		// unit keyed by its def-site. Two conditionally-defined classes with
		// the same textual name share `baseFqName` but get distinct identity
		// names like `dispatch.config.Secret@24:4` and `…@38:4`, so the
		// PyClassType registry never silently merges them.
		SourceCodeLocation classLoc = support.getLocation(pctx);
		String fqName = baseFqName + "@" + classLoc.getLine() + ":" + classLoc.getCol();
		PyClassUnit cu = new PyClassUnit(classLoc, ctx.program(), fqName, baseFqName, false);
		PyClassType.register(fqName, cu);
		ctx.currentUnit(cu);
		PyCFG classInit = new PyCFG(support.buildInitClassCFGDescriptor(SyntheticLocation.INSTANCE));
		cu.addCodeMember(classInit);

		List<ArgumentContext> superclasses = pctx.arglist() != null ? new ArrayList<>(pctx.arglist().argument())
				: new ArrayList<>();
		// Resolve ancestors by Python-visible name. If a simple name resolves
		// to multiple def-sites (conditional class redefinition), add ALL
		// matching units as ancestors — a sound over-approximation that lets
		// downstream subclass checks succeed against any possible parent.
		for (ArgumentContext superclass : superclasses) {
			String superClassName = ctx.imports().getOrDefault(superclass.getText(), superclass.getText());
			Set<CompilationUnit> matches = new LinkedHashSet<>();
			for (Unit programCu : ctx.program().getUnits()) {
				if (!(programCu instanceof CompilationUnit programCompUnit))
					continue;
				String candidateIdentity = programCu.getName();
				String candidateBase = (programCu instanceof PyClassUnit pcu) ? pcu.getBaseName()
						: candidateIdentity;
				if (candidateIdentity.equals(superClassName) || candidateBase.equals(superClassName))
					matches.add(programCompUnit);
				else if (ctx.currentModule() != null
						&& candidateBase.equals(ctx.currentModule().getName() + "." + superClassName))
					matches.add(programCompUnit);
				else if (candidateBase.endsWith("." + superClassName))
					matches.add(programCompUnit);
			}
			for (CompilationUnit match : matches)
				cu.addAncestor(match);
		}
		if (cu.getImmediateAncestors().isEmpty()) {
			if (ctx.objectUnit() != null)
				cu.addAncestor(ctx.objectUnit());
			else
				LOG.warn("builtins.object is not available; class '{}' will have no ancestor. "
						+ "Ensure toLiSAProgram() completes library loading before parsing.", fqName);
		}
		LOG.debug("DEBUG visitClassdef: class '{}' (base '{}') ancestors = {}", fqName, baseFqName,
				cu.getImmediateAncestors());
		// Register the class unit in the program BEFORE parsing the body so
		// super() detection in method bodies can look up the class by name.
		ctx.program().addUnit(cu);
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> classInitBody = parseClassBody(pctx.suite());
		classInit.getNodeList().mergeWith(classInitBody.getMiddle());
		if (pctx.suite().stmt().isEmpty()) {
			NoOp noOp = new NoOp(classInit, SyntheticLocation.INSTANCE);
			classInit.addNode(noOp, true);
			classInit.addEdge(new SequentialEdge(noOp, classInitBody.getLeft()));
		} else
			classInit.addNodeIfNotPresent(classInitBody.getLeft(), true);
		ctx.currentUnit(previous);
		Expression target;
		if (ctx.currentUnit() instanceof ModuleUnit pmu) {
			target = support.makeScopedAttributeRef(pmu, name, support.getLocation(pctx));
		} else if (ctx.currentUnit() instanceof ClassUnit) {
			target = new AttributeAccess(ctx.currentCFG(), support.getLocation(pctx),
					new VariableRef(ctx.currentCFG(), support.getLocation(pctx), ParserContext.SELF_PARAM_NAME),
					name);
		} else {
			ctx.declareNameInCurrentScope(name);
			target = new VariableRef(ctx.currentCFG(), support.getLocation(pctx), name);
		}
		PyAssign classAssign = new PyAssign(ctx.currentCFG(), support.getLocation(pctx), target,
				new ImportClass(ctx.currentCFG(), SyntheticLocation.INSTANCE, name, cu));
		block.addNode(classAssign);

		return Triple.of(classAssign, block, classAssign);
	}

	private Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> parseClassBody(
			SuiteContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Statement first = null;
		Statement last = null;
		ctx.enterLocalScope();
		try {
			for (StmtContext stmt : pctx.stmt()) {
				if (stmt.simple_stmt() != null) {
					Statement s = parseField(stmt.simple_stmt());
					block.addNode(s);
					if (first == null)
						first = s;
					if (last != null)
						block.addEdge(new SequentialEdge(last, s));
					last = s;
				} else if (stmt.compound_stmt().funcdef() != null) {
					Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = ctx.def()
							.visitFuncdef(stmt.compound_stmt().funcdef());
					if (r.getLeft() != null) {
						if (first == null)
							first = r.getLeft();
						block.mergeWith(r.getMiddle());
						if (last != null)
							block.addEdge(new SequentialEdge(last, r.getLeft()));
						last = r.getRight();
					}
				} else if (stmt.compound_stmt().async_stmt() != null) {
					Object async = ctx.stmt().visitAsync_stmt(stmt.compound_stmt().async_stmt());
					if (async instanceof Triple<?, ?, ?> triple
							&& triple.getLeft() instanceof Statement left
							&& triple.getMiddle() instanceof NodeList<?, ?, ?> middle
							&& triple.getRight() instanceof Statement right) {
						@SuppressWarnings("unchecked")
						NodeList<CFG, Statement, Edge> middleBlock = (NodeList<CFG, Statement, Edge>) middle;
						if (first == null)
							first = left;
						block.mergeWith(middleBlock);
						if (last != null)
							block.addEdge(new SequentialEdge(last, left));
						last = right;
					}
				} else if (stmt.compound_stmt().decorated() != null) {
					Object decorated = ctx.def().visitDecorated(stmt.compound_stmt().decorated());
					if (decorated instanceof Triple<?, ?, ?> triple
							&& triple.getLeft() instanceof Statement left
							&& triple.getMiddle() instanceof NodeList<?, ?, ?> middle
							&& triple.getRight() instanceof Statement right) {
						@SuppressWarnings("unchecked")
						NodeList<CFG, Statement, Edge> middleBlock = (NodeList<CFG, Statement, Edge>) middle;
						if (first == null)
							first = left;
						block.mergeWith(middleBlock);
						if (last != null)
							block.addEdge(new SequentialEdge(last, left));
						last = right;
					}
				}
			}
		} finally {
			ctx.exitLocalScope();
		}
		Ret ret = new Ret(ctx.currentCFG(), SyntheticLocation.INSTANCE);
		block.addNode(ret);
		if (last != null)
			block.addEdge(new SequentialEdge(last, ret));
		else
			first = ret;
		last = ret;
		return Triple.of(first, block, last);
	}

	private Statement parseField(
			Simple_stmtContext st) {
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> simple = ctx.stmt().visitSimple_stmt(st);
		Collection<Statement> nodes = simple.getMiddle().getNodes();
		if (nodes.size() != 1)
			throw new UnsupportedStatementException("Expected a single statement, got " + nodes.size());
		return nodes.iterator().next();
	}
}
