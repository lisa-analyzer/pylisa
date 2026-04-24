package it.unive.pylisa.frontend.definition;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.Async_funcdefContext;
import it.unive.pylisa.antlr.Python3Parser.ClassdefContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratedContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorsContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.antlr.Python3Parser.TfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarpositionalContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.statement.ImportClass;
import it.unive.pylisa.cfg.statement.ImportFunction;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python definitions: {@code def}, {@code class}, {@code async def},
 * decorator application, and their parameter lists. One of three sibling
 * category visitors introduced in Chunk 2; shares state with
 * {@link ParserContext} and cross-dispatches via {@code ctx.expr()} /
 * {@code ctx.stmt()}.
 */
public final class DefinitionVisitor extends Python3ParserBaseVisitor<Object> {

	private static final Logger LOG = LogManager.getLogger(DefinitionVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public DefinitionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	@Override
	public Object visitDecorated(
			DecoratedContext pctx) {
		if (pctx.decorators().isEmpty()) {
			return support.unsupported(pctx, "Expecting a DecoratorsContext in DecoratedContext");
		}
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Expression result = null;
		Expression innerAssign = null;
		if (pctx.classdef() != null) {
			return support.unsupported(pctx, "Class Decorators are not supported yet");
		} else if (pctx.async_funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = visitAsync_funcdef(
					pctx.async_funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(pctx.decorators(), func);
			} else {
				return support.unsupported(pctx, "Expecting a PyAssign while parsing async_funcDef");
			}
		} else if (pctx.funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = visitFuncdef(pctx.funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(pctx.decorators(), func);
			} else {
				return support.unsupported(pctx, "Expecting a PyAssign while parsing funcDef");
			}
		} else {
			return support.unsupported(pctx, "Expecting {'def', 'class', 'async'} after decorators");
		}
		PyAssign pyAssign = new PyAssign(ctx.currentCFG(), support.getLocation(pctx), innerAssign, result);
		block.addNode(pyAssign);
		return Triple.of(pyAssign, block, pyAssign);
	}

	@Override
	public FunctionApply visitDecorator(
			DecoratorContext pctx) {
		if (pctx.dotted_name() == null) {
			throw new UnsupportedOperationException("Expecting a Dotted_nameContext in a DecoratorContext.");
		}
		Expression result = ctx.stmt().visitDotted_name(pctx.dotted_name());
		/*
		 * If the result is a VariableRef, e.g. @f() -> VariableRef(f), it means
		 * we are in the current scope — no need to add a parameter in the
		 * function.
		 */

		if (result instanceof VariableRef) {
			List<Expression> params = new ArrayList<>();
			if (pctx.arglist() != null)
				for (ArgumentContext arg : pctx.arglist().argument())
					params.add(ctx.expr().visitArgument(arg));
			params = support.convertAssignmentsToByNameParameters(params);
			return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), result,
					params.toArray(Expression[]::new));
		}
		List<Expression> params = new ArrayList<>();
		String varName = pctx.dotted_name().children.get(0).getText();
		params.add(support.makeRef(varName, support.getLocation(pctx)));
		if (pctx.arglist() != null)
			for (ArgumentContext arg : pctx.arglist().argument())
				params.add(ctx.expr().visitArgument(arg));
		params = support.convertAssignmentsToByNameParameters(params);
		return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), result,
				params.toArray(Expression[]::new));
	}

	public Expression visitDecorators(
			DecoratorsContext pctx,
			Expression decoratedFunction) {
		Expression result = decoratedFunction;

		List<DecoratorContext> decorators = pctx.decorator();

		for (int i = decorators.size() - 1; i >= 0; i--) {
			DecoratorContext decorCtx = decorators.get(i);
			FunctionApply decorator = visitDecorator(decorCtx);

			if (result == null) {
				result = decorator;
			} else {
				// @f(args) → f(args)(func): decorator is a call, use it as
				// target (double-wrap)
				// @f → f(func): decorator IS the callable, extract target
				// (single-wrap)
				boolean hasExplicitParens = decorCtx.OPEN_PAREN() != null;
				Expression callTarget = hasExplicitParens
						? decorator
						: decorator.getSubExpressions()[0];
				result = new FunctionApply(
						ctx.currentCFG(),
						support.getLocation(pctx),
						callTarget,
						List.of(result).toArray(Expression[]::new));
			}
		}

		return result;
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitAsync_funcdef(
			Async_funcdefContext pctx) {
		support.unsound(pctx, "async def treated as def");
		return visitFuncdef(pctx.funcdef());
	}

	@Override
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
			for (PyParameter parameter : visitParameters(pctx.parameters()))
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

	@Override
	public PyParameter[] visitParameters(
			ParametersContext pctx) {
		if (pctx.typedargslist() == null)
			return new PyParameter[0];
		return visitTypedargslist(pctx.typedargslist());
	}

	@Override
	public PyParameter[] visitTypedargslist(
			TypedargslistContext pctx) {
		List<PyParameter> pars = new LinkedList<>();
		for (TypedargContext typedArg : pctx.typedarg())
			if (pars.isEmpty())
				if (ctx.currentUnit() instanceof ClassUnit) {
					pars.add(new PyParameter(support.getLocation(typedArg), typedArg.tfpdef().NAME().getText(),
							new ReferenceType(PyClassType.register(ctx.currentUnit().getName(),
									(ClassUnit) ctx.currentUnit()))));
				} else
					pars.add(visitTypedarg(typedArg));
			else
				pars.add(visitTypedarg(typedArg));

		if (pctx.starargs() != null)
			pars.addAll(Arrays.asList(visitStarargs(pctx.starargs())));

		if (pctx.varkw() != null)
			pars.add(visitVarkw(pctx.varkw()));

		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter[] visitStarargs(
			Python3Parser.StarargsContext pctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (pctx.varpositional() != null) {
			VarpositionalContext def = pctx.varpositional();
			pars.add(new VarPositionalParameter(support.getLocation(def), def.tfpdef().NAME().getText()));
		}

		if (pctx.typedarg() != null) {
			List<TypedargContext> def = pctx.typedarg();
			for (TypedargContext typedArg : def)
				pars.add(new KeywordOnlyParameter(visitTypedarg(typedArg)));
		}
		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter visitVarkw(
			Python3Parser.VarkwContext pctx) {
		return new VarKeywordParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText());
	}

	@Override
	public PyParameter visitTypedarg(
			TypedargContext pctx) {
		String typeHint = null;
		if (pctx.tfpdef().test() != null) {
			typeHint = ctx.expr().visitTest(pctx.tfpdef().test()).toString();
		}
		if (pctx.test() == null)
			return new PyParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText(), Untyped.INSTANCE, null,
					null, typeHint);
		else
			return new PyParameter(support.getLocation(pctx), pctx.tfpdef().NAME().getText(), Untyped.INSTANCE,
					ctx.expr().visitTest(pctx.test()), null, typeHint);
	}

	@Override
	public PyParameter visitTfpdef(
			TfpdefContext pctx) {
		return new PyParameter(support.getLocation(pctx), pctx.NAME().getText(), Untyped.INSTANCE);
	}

	@Override
	public Object visitVarargslist(
			VarargslistContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public String visitVfpdef(
			VfpdefContext pctx) {
		return pctx.NAME().getText();
	}

	@Override
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

	// === helpers owned by this visitor ===

	public CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl,
			Unit unit) {
		PyParameter[] cfgArgs = visitParameters(funcDecl.parameters());
		return new CodeMemberDescriptor(support.getLocation(funcDecl), unit, false, "$call", cfgArgs);
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
					Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = visitFuncdef(
							stmt.compound_stmt().funcdef());
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
					Object decorated = visitDecorated(stmt.compound_stmt().decorated());
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
