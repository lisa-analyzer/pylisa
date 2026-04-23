package it.unive.pylisa.frontend;

import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
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
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import org.apache.commons.lang3.tuple.Triple;

public abstract class PyDefinitionVisitorBase extends PyStatementVisitorBase {

	public PyDefinitionVisitorBase(
			String filePath,
			boolean notebook) {
		super(filePath, notebook);
	}

	public PyDefinitionVisitorBase(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	public PyDefinitionVisitorBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	public PyDefinitionVisitorBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot) {
		super(filePath, notebook, cellOrder, sourceRoot);
	}

	/*
	 * decorated : decorators (classdef | funcdef | async_funcdef) ;
	 * @param ctx the parse tree
	 * @return
	 */
	@Override
	public Object visitDecorated(
			DecoratedContext ctx) {
		if (ctx.decorators().isEmpty()) {
			return unsupported(ctx, "Expecting a DecoratorsContext in DecoratedContext");
		}
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Expression result = null;
		Expression innerAssign = null;
		if (ctx.classdef() != null) {
			return unsupported(ctx, "Class Decorators are not supported yet");
		} else if (ctx.async_funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = visitAsync_funcdef(
					ctx.async_funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(ctx.decorators(), func);
			} else {
				return unsupported(ctx, "Expecting a PyAssign while parsing async_funcDef");
			}
		} else if (ctx.funcdef() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> funcDef = visitFuncdef(ctx.funcdef());
			if (funcDef.getLeft() instanceof PyAssign pa) {
				innerAssign = pa.getLeft();
				Expression func = pa.getRight();
				result = visitDecorators(ctx.decorators(), func);
			} else {
				return unsupported(ctx, "Expecting a PyAssign while parsing funcDef");
			}
		} else {
			return unsupported(ctx, "Expecting {'def', 'class', 'async'} after decorators");
		}
		PyAssign pyAssign = new PyAssign(this.ctx.currentCFG(), getLocation(ctx), innerAssign, result);
		block.addNode(pyAssign);
		return Triple.of(pyAssign, block, pyAssign);
	}

	@Override
	public FunctionApply visitDecorator(
			DecoratorContext ctx) {
		if (ctx.dotted_name() == null) {
			throw new UnsupportedOperationException("Expecting a Dotted_nameContext in a DecoratorContext.");
		}
		// no-paren decorator like @asynccontextmanager — handled below with
		// empty params
		Expression result = visitDotted_name(ctx.dotted_name());
		/*
		 * If the result is a VariableRef, for example, @f() -> VariableRef(f),
		 * it means we are in the current scope. No need to add a parameter in
		 * the function.
		 */

		if (result instanceof VariableRef) {
			List<Expression> params = new ArrayList<>();
			if (ctx.arglist() != null)
				for (ArgumentContext arg : ctx.arglist().argument())
					params.add(visitArgument(arg));
			params = convertAssignmentsToByNameParameters(params);
			return new FunctionApply(this.ctx.currentCFG(), getLocation(ctx), result,
					params.toArray(Expression[]::new));
		}
		List<Expression> params = new ArrayList<>();
		String varName = ctx.dotted_name().children.get(0).getText();
		params.add(makeRef(varName, getLocation(ctx)));
		if (ctx.arglist() != null)
			for (ArgumentContext arg : ctx.arglist().argument())
				params.add(visitArgument(arg));
		params = convertAssignmentsToByNameParameters(params);
		return new FunctionApply(this.ctx.currentCFG(), getLocation(ctx), result, params.toArray(Expression[]::new));
	}

	public Expression visitDecorators(
			DecoratorsContext ctx,
			Expression decoratedFunction) {
		Expression result = decoratedFunction;

		List<DecoratorContext> decorators = ctx.decorator();

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
						this.ctx.currentCFG(),
						getLocation(ctx),
						callTarget,
						List.of(result).toArray(Expression[]::new));
			}
		}

		return result;
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitAsync_funcdef(
			Async_funcdefContext ctx) {
		unsound(ctx, "async def treated as def");
		return visitFuncdef(ctx.funcdef());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFuncdef(
			FuncdefContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		PyCFG oldCFG = this.ctx.currentCFG();
		Collection<ControlFlowStructure> oldCfs = this.ctx.cfs();

		FunctionUnit unit = new FunctionUnit(getLocation(ctx), this.ctx.program(),
				this.ctx.currentUnit() + "." + ctx.NAME().getText(),
				false);
		PyFunctionType.register(unit.getName(), unit);
		PyCFG newCFG = new PyCFG(buildCFGDescriptor(ctx, unit));
		this.ctx.currentCFG(newCFG);
		unit.setFunction(newCFG);
		unit.addCodeMember(newCFG);
		this.ctx.program().addUnit(unit);
		this.ctx.cfs(new HashSet<>());
		Unit prevUnit = this.ctx.currentUnit();
		this.ctx.currentUnit(unit);
		enterLocalScope();
		try {
			for (PyParameter parameter : visitParameters(ctx.parameters()))
				declareNameInCurrentScope(parameter.getName());
		} catch (UnsupportedStatementException e) {
			if (!continueOnUnsupportedStatement)
				throw e;
			log.warn("[PyLiSA] Skipping unsupported parameter in " + unit.getName()
					+ ": " + e.getMessage());
			// this.ctx.currentCFG() is still newCFG; the finally below will
			// restore it
		}
		try {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = visitSuite(ctx.suite());
			this.ctx.currentUnit(prevUnit);
			this.ctx.currentCFG().getNodeList().mergeWith(r.getMiddle());
			this.ctx.currentCFG().getEntrypoints().add(r.getLeft());
			addRetNodesToCurrentCFG();
			this.ctx.cfs().forEach(this.ctx.currentCFG().getDescriptor()::addControlFlowStructure);
			this.ctx.currentCFG().simplify();
		} catch (Exception e) {
			// Finalize the function CFG even if parsing partially failed
			addRetNodesToCurrentCFG();
			this.ctx.currentUnit(prevUnit);
			throw e;
		} finally {
			exitLocalScope();
			this.ctx.currentCFG(oldCFG);
			this.ctx.cfs(oldCfs);
		}
		Expression target;
		if (this.ctx.currentUnit() instanceof ModuleUnit pmu) {
			target = makeScopedAttributeRef(pmu, ctx.NAME().getText(), getLocation(ctx));
		} else if (this.ctx.currentUnit() instanceof ClassUnit cu) {
			target = makeScopedAttributeRef(cu, ctx.NAME().getText(), getLocation(ctx));
		} else {
			declareNameInCurrentScope(ctx.NAME().getText());
			target = new VariableRef(this.ctx.currentCFG(), getLocation(ctx), ctx.NAME().getText());
		}
		PyAssign funcAssign = new PyAssign(this.ctx.currentCFG(), getLocation(ctx), target,
				new ImportFunction(this.ctx.currentCFG(), SyntheticLocation.INSTANCE, unit.getName(), unit));
		block.addNode(funcAssign);
		return Triple.of(funcAssign, block, funcAssign);
	}

	@Override
	public PyParameter[] visitParameters(
			ParametersContext ctx) {
		if (ctx.typedargslist() == null)
			return new PyParameter[0];
		return visitTypedargslist(ctx.typedargslist());
	}

	@Override
	public PyParameter[] visitTypedargslist(
			TypedargslistContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		for (TypedargContext typedArg : ctx.typedarg())
			if (pars.isEmpty())
				if (this.ctx.currentUnit() instanceof ClassUnit) {
					pars.add(new PyParameter(getLocation(typedArg), typedArg.tfpdef().NAME().getText(),
							new ReferenceType(PyClassType.register(this.ctx.currentUnit().getName(),
									(ClassUnit) this.ctx.currentUnit()))));
				} else
					pars.add(visitTypedarg(typedArg));
			else
				pars.add(visitTypedarg(typedArg));

		if (ctx.starargs() != null)
			pars.addAll(Arrays.asList(visitStarargs(ctx.starargs())));

		if (ctx.varkw() != null)
			pars.add(visitVarkw(ctx.varkw()));

		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter[] visitStarargs(
			Python3Parser.StarargsContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (ctx.varpositional() != null) {
			VarpositionalContext def = ctx.varpositional();
			pars.add(new VarPositionalParameter(getLocation(def), def.tfpdef().NAME().getText()));
		}

		if (ctx.typedarg() != null) {
			List<TypedargContext> def = ctx.typedarg();
			for (TypedargContext typedArg : def)
				pars.add(new KeywordOnlyParameter(visitTypedarg(typedArg)));
		}
		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter visitVarkw(
			Python3Parser.VarkwContext ctx) {
		return new VarKeywordParameter(getLocation(ctx), ctx.tfpdef().NAME().getText());
	}

	@Override
	public PyParameter visitTypedarg(
			TypedargContext ctx) {
		String typeHint = null;
		if (ctx.tfpdef().test() != null) {
			typeHint = visitTest(ctx.tfpdef().test()).toString();
		}
		if (ctx.test() == null)
			return new PyParameter(getLocation(ctx), ctx.tfpdef().NAME().getText(), Untyped.INSTANCE, null, null,
					typeHint);
		else
			return new PyParameter(getLocation(ctx), ctx.tfpdef().NAME().getText(), Untyped.INSTANCE,
					visitTest(ctx.test()), null, typeHint);
	}

	@Override
	public PyParameter visitTfpdef(
			TfpdefContext ctx) {
		return new PyParameter(getLocation(ctx), ctx.NAME().getText(), Untyped.INSTANCE);
	}

	@Override
	public Object visitVarargslist(
			VarargslistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public String visitVfpdef(
			VfpdefContext ctx) {
		return ctx.NAME().getText();
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitClassdef(
			ClassdefContext ctx) {

		// attach classInit to the class unit
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Unit previous = this.ctx.currentUnit();
		String name = ctx.NAME().getSymbol().getText();
		String baseFqName = previous.getName() + "." + name;
		// Allocation-site abstraction: each `class` statement mints a fresh
		// unit keyed by its def-site. Two conditionally-defined classes with
		// the same textual name (e.g. `if cond: class Secret: … else: class
		// Secret: …`) share `baseFqName` but get distinct identity names like
		// `dispatch.config.Secret@24:4` and `…@38:4`, so the PyClassType
		// registry never silently merges them. Name-level resolution
		// (this.ctx.imports(), attribute access, inheritance) goes through
		// {@link PyClassType#lookupAllByBaseName} which returns the set of
		// def-sites for a given qualified name.
		SourceCodeLocation classLoc = getLocation(ctx);
		String fqName = baseFqName + "@" + classLoc.getLine() + ":" + classLoc.getCol();
		PyClassUnit cu = new PyClassUnit(classLoc, this.ctx.program(), fqName, baseFqName, false);
		PyClassType.register(fqName, cu);
		this.ctx.currentUnit(cu);
		PyCFG classInit = new PyCFG(buildInitClassCFGDescriptor(SyntheticLocation.INSTANCE));
		cu.addCodeMember(classInit);

		ArrayList<ArgumentContext> superclasses = ctx.arglist() != null ? new ArrayList<>(ctx.arglist().argument())
				: new ArrayList<>();
		// Resolve ancestors by Python-visible name. If a simple name resolves
		// to multiple def-sites (conditional class redefinition), we add ALL
		// matching units as ancestors — a sound over-approximation that lets
		// downstream subclass checks succeed against any possible parent.
		for (ArgumentContext superclass : superclasses) {
			String superClassName = this.ctx.imports().getOrDefault(superclass.getText(), superclass.getText());
			java.util.Set<it.unive.lisa.program.CompilationUnit> matches = new java.util.LinkedHashSet<>();
			for (Unit programCu : this.ctx.program().getUnits()) {
				if (!(programCu instanceof it.unive.lisa.program.CompilationUnit programCompUnit))
					continue;
				String candidateIdentity = programCu.getName();
				String candidateBase = (programCu instanceof PyClassUnit pcu) ? pcu.getBaseName()
						: candidateIdentity;
				if (candidateIdentity.equals(superClassName) || candidateBase.equals(superClassName))
					matches.add(programCompUnit);
				else if (this.ctx.currentModule() != null
						&& candidateBase.equals(this.ctx.currentModule().getName() + "." + superClassName))
					matches.add(programCompUnit);
				else if (candidateBase.endsWith("." + superClassName))
					matches.add(programCompUnit);
			}
			for (it.unive.lisa.program.CompilationUnit match : matches)
				cu.addAncestor(match);
		}
		if (cu.getImmediateAncestors().isEmpty()) {
			if (this.ctx.objectUnit() != null)
				cu.addAncestor(this.ctx.objectUnit());
			else
				log.warn("builtins.object is not available; class '{}' will have no ancestor. "
						+ "Ensure toLiSAProgram() completes library loading before parsing.", fqName);
		}
		log.debug("DEBUG visitClassdef: class '{}' (base '{}') ancestors = {}", fqName, baseFqName,
				cu.getImmediateAncestors());
		// Register the class unit in the this.ctx.program() BEFORE parsing the
		// body so
		// that
		// super() detection in method bodies can look up the class by name.
		this.ctx.program().addUnit(cu);
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> classInitBody = parseClassBody(ctx.suite());
		classInit.getNodeList().mergeWith(classInitBody.getMiddle());
		if (ctx.suite().stmt().isEmpty()) {
			NoOp noOp = new NoOp(classInit, SyntheticLocation.INSTANCE);
			classInit.addNode(noOp, true);
			classInit.addEdge(new SequentialEdge(noOp, classInitBody.getLeft()));
		} else
			classInit.addNodeIfNotPresent(classInitBody.getLeft(), true);
		this.ctx.currentUnit(previous);
		Expression target;
		if (this.ctx.currentUnit() instanceof ModuleUnit pmu) {
			target = makeScopedAttributeRef(pmu, name, getLocation(ctx));
		} else if (this.ctx.currentUnit() instanceof ClassUnit) {
			target = new AttributeAccess(this.ctx.currentCFG(), getLocation(ctx),
					new VariableRef(this.ctx.currentCFG(), getLocation(ctx), SELF_PARAM_NAME), name);
		} else {
			declareNameInCurrentScope(name);
			target = new VariableRef(this.ctx.currentCFG(), getLocation(ctx), name);
		}
		PyAssign classAssign = new PyAssign(this.ctx.currentCFG(), getLocation(ctx), target,
				new ImportClass(this.ctx.currentCFG(), SyntheticLocation.INSTANCE, name, cu));
		block.addNode(classAssign);
		// Ret ret = new Ret(this.ctx.currentCFG(), SyntheticLocation.INSTANCE);
		// block.addNode(ret);
		// block.addEdge(new SequentialEdge(classAssign, ret));

		return Triple.of(classAssign, block, classAssign);
	}

	private Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> parseClassBody(
			SuiteContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null;
		Statement last = null;
		enterLocalScope();
		try {
			for (StmtContext stmt : ctx.stmt()) {
				if (stmt.simple_stmt() != null) {
					it.unive.lisa.program.cfg.statement.Statement s = parseField(stmt.simple_stmt());
					block.addNode(s);
					if (first == null) {
						first = s;
					}
					if (last != null) {
						block.addEdge(new SequentialEdge(last, s));
					}
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
					Object async = visitAsync_stmt(stmt.compound_stmt().async_stmt());
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
			exitLocalScope();
		}
		Ret ret = new Ret(this.ctx.currentCFG(), SyntheticLocation.INSTANCE);
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
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> simple = visitSimple_stmt(st);
		Collection<Statement> nodes = simple.getMiddle().getNodes();
		if (nodes.size() != 1)
			throw new UnsupportedStatementException("Expected a single statement, got " + nodes.size());
		return nodes.iterator().next();
	}
}
