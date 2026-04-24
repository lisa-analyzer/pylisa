package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.Async_funcdefContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.ImportFunction;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python function definitions ({@code funcdef}, {@code async_funcdef}).
 * Extracted from {@link DefinitionVisitor} in Chunk 5. The {@code try/finally}
 * state-restore around body parsing is preserved verbatim to keep the prior fix
 * against partial-CFG leakage on exception (see MEMORY).
 */
public final class FunctionDefinitionVisitor {

	private static final Logger LOG = LogManager.getLogger(FunctionDefinitionVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public FunctionDefinitionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitAsync_funcdef(
			Async_funcdefContext pctx) {
		support.unsound(pctx, "async def treated as def");
		return visitFuncdef(pctx.funcdef());
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFuncdef(
			FuncdefContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		PyCFG oldCFG = ctx.currentCFG();
		Collection<ControlFlowStructure> oldCfs = ctx.cfs();

		FunctionUnit unit = new FunctionUnit(support.getLocation(pctx), ctx.program(),
				ctx.currentUnit() + "." + pctx.NAME().getText(),
				false);
		PyFunctionType.register(unit.getName(), unit);
		PyCFG newCFG = new PyCFG(buildCFGDescriptor(pctx, unit));
		ctx.currentCFG(newCFG);
		unit.setFunction(newCFG);
		unit.addCodeMember(newCFG);
		ctx.program().addUnit(unit);
		ctx.cfs(new HashSet<>());
		Unit prevUnit = ctx.currentUnit();
		ctx.currentUnit(unit);
		ctx.enterLocalScope();
		try {
			for (PyParameter parameter : ctx.def().visitParameters(pctx.parameters()))
				ctx.declareNameInCurrentScope(parameter.getName());
		} catch (UnsupportedStatementException e) {
			if (!ctx.continueOnUnsupportedStatement())
				throw e;
			LOG.warn("[PyLiSA] Skipping unsupported parameter in " + unit.getName()
					+ ": " + e.getMessage());
			// ctx.currentCFG() is still newCFG; the finally below restores it.
		}
		try {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = ctx.stmt().visitSuite(pctx.suite());
			ctx.currentUnit(prevUnit);
			ctx.currentCFG().getNodeList().mergeWith(r.getMiddle());
			ctx.currentCFG().getEntrypoints().add(r.getLeft());
			support.addRetNodesToCurrentCFG();
			ctx.cfs().forEach(ctx.currentCFG().getDescriptor()::addControlFlowStructure);
			ctx.currentCFG().simplify();
		} catch (Exception e) {
			// Finalize the function CFG even if parsing partially failed
			support.addRetNodesToCurrentCFG();
			ctx.currentUnit(prevUnit);
			throw e;
		} finally {
			ctx.exitLocalScope();
			ctx.currentCFG(oldCFG);
			ctx.cfs(oldCfs);
		}
		Expression target;
		if (ctx.currentUnit() instanceof ModuleUnit pmu) {
			target = support.makeScopedAttributeRef(pmu, pctx.NAME().getText(), support.getLocation(pctx));
		} else if (ctx.currentUnit() instanceof ClassUnit cu) {
			target = support.makeScopedAttributeRef(cu, pctx.NAME().getText(), support.getLocation(pctx));
		} else {
			ctx.declareNameInCurrentScope(pctx.NAME().getText());
			target = new VariableRef(ctx.currentCFG(), support.getLocation(pctx), pctx.NAME().getText());
		}
		PyAssign funcAssign = new PyAssign(ctx.currentCFG(), support.getLocation(pctx), target,
				new ImportFunction(ctx.currentCFG(), SyntheticLocation.INSTANCE, unit.getName(), unit));
		block.addNode(funcAssign);
		return Triple.of(funcAssign, block, funcAssign);
	}

	public CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl,
			Unit unit) {
		PyParameter[] cfgArgs = ctx.def().visitParameters(funcDecl.parameters());
		return new CodeMemberDescriptor(support.getLocation(funcDecl), unit, false, "$call", cfgArgs);
	}
}
