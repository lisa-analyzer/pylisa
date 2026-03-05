package it.unive.pylisa;

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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;
import org.antlr.v4.runtime.tree.ParseTree;
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
		Statement first = null, last = null;
		Expression result = null;
		String name = "";
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
		Expression target = null;
		if (currentUnit instanceof ModuleUnit pmu) {
			target = new PythonUnitAttributeAccessRef(this.currentCFG, getLocation(ctx), pmu,
					new Global(getLocation(ctx), pmu, name, false));
		} else {
			target = new AttributeAccess(this.currentCFG, getLocation(ctx),
					new VariableRef(this.currentCFG, getLocation(ctx), "$self"), name);
		}
		PyAssign pyAssign = new PyAssign(currentCFG, getLocation(ctx), innerAssign, result);
		first = pyAssign;
		block.addNode(pyAssign);
		last = result;
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
			String target = ((VariableRef) result).getName();
			if (ctx.arglist() != null)
				for (ArgumentContext arg : ctx.arglist().argument())
					params.add(visitArgument(arg));
			params = convertAssignmentsToByNameParameters(params);
			return new FunctionApply(currentCFG, getLocation(ctx), result, params.toArray(Expression[]::new));
			// return new UnresolvedCall(currentCFG, getLocation(ctx),
			// CallType.STATIC, null, target,
			// params.toArray(Expression[]::new));
		}
		List<Expression> params = new ArrayList<>();
		String varName = ctx.dotted_name().children.get(0).getText();
		params.add(makeRef(varName, getLocation(ctx)));
		if (ctx.arglist() != null)
			for (ArgumentContext arg : ctx.arglist().argument())
				params.add(visitArgument(arg));
		params = convertAssignmentsToByNameParameters(params);

		List<ParseTree> trees = ctx.dotted_name().children.subList(1, ctx.dotted_name().children.size());
		String target = trees.stream()
				.map(ParseTree::getText)
				.filter(text -> !text.equals("."))
				.collect(Collectors.joining("."));
		return new FunctionApply(currentCFG, getLocation(ctx), result, params.toArray(Expression[]::new));
		// return new UnresolvedCall(currentCFG, getLocation(ctx),
		// CallType.UNKNOWN, null, target,
		// params.toArray(Expression[]::new));
		// return new AnnotationMember(ctx.dotted_name().getText(), new
		// DecoratedAnnotation(params, uc));
	}

	public Expression visitDecorators(
			DecoratorsContext ctx,
			Expression decoratedFunction) {
		Expression result = decoratedFunction;

		List<DecoratorContext> decorators = ctx.decorator();

		for (int i = decorators.size() - 1; i >= 0; i--) {
			FunctionApply decorator = visitDecorator(decorators.get(i));

			if (result == null) {
				result = decorator;
			} else {
				result = new FunctionApply(
						currentCFG,
						getLocation(ctx),
						decorator,
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
		PyCFG oldCFG = currentCFG;
		Collection<ControlFlowStructure> oldCfs = cfs;

		FunctionUnit unit = new FunctionUnit(getLocation(ctx), program, currentUnit + "." + ctx.NAME().getText(),
				false);
		PyFunctionType.register(unit.getName(), unit);
		PyCFG newCFG = currentCFG = new PyCFG(buildCFGDescriptor(ctx, unit));
		unit.setFunction(newCFG);
		unit.addCodeMember(newCFG);
		program.addUnit(unit);
		cfs = new HashSet<>();
		Unit prevUnit = currentUnit;
		currentUnit = unit;
		try {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = visitSuite(ctx.suite());
			currentUnit = prevUnit;
			currentCFG.getNodeList().mergeWith(r.getMiddle());
			currentCFG.getEntrypoints().add(r.getLeft());
			addRetNodesToCurrentCFG();
			cfs.forEach(currentCFG.getDescriptor()::addControlFlowStructure);
			currentCFG.simplify();
		} catch (Exception e) {
			// Finalize the function CFG even if parsing partially failed
			addRetNodesToCurrentCFG();
			currentUnit = prevUnit;
			throw e;
		} finally {
			currentCFG = oldCFG;
			cfs = oldCfs;
		}
		Expression target;
		if (currentUnit instanceof ModuleUnit pmu) {
			target = new PythonUnitAttributeAccessRef(this.currentCFG, getLocation(ctx), pmu,
					new Global(getLocation(ctx), pmu, ctx.NAME().getText(), false));
		} else {
			target = new AttributeAccess(this.currentCFG, getLocation(ctx),
					new VariableRef(this.currentCFG, getLocation(ctx), "$self"), unit.getName());
		}
		PyAssign funcAssign = new PyAssign(this.currentCFG, getLocation(ctx), target,
				new ImportFunction(currentCFG, SyntheticLocation.INSTANCE, unit.getName(), unit));
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
				if (currentUnit instanceof ClassUnit) {
					pars.add(new PyParameter(getLocation(typedArg), typedArg.tfpdef().NAME().getText(),
							new ReferenceType(PyClassType.register(currentUnit.getName(), (ClassUnit) currentUnit))));
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
		Unit previous = this.currentUnit;
		String name = ctx.NAME().getSymbol().getText();
		String fqName = previous.getName() + "." + name;
		ClassUnit cu = new ClassUnit(new SourceCodeLocation(name, 0, 0), program, fqName, true);
		this.currentUnit = cu;
		PyClassType.register(fqName, cu);
		PyCFG classInit = new PyCFG(buildInitClassCFGDesriptor(SyntheticLocation.INSTANCE));
		cu.addCodeMember(classInit);
		// TODO inheritance

		ArrayList<ArgumentContext> superclasses = ctx.arglist() != null ? new ArrayList<>(ctx.arglist().argument())
				: new ArrayList<>();
		// parse anchestors
		for (ArgumentContext superclass : superclasses) {
			// if exists a class unit in the program with name
			// superclass.getText(): add it
			// to the anchestors
			String superClassName = imports.getOrDefault(superclass.getText(), superclass.getText());
			for (Unit programCu : this.program.getUnits())
				if (programCu instanceof it.unive.lisa.program.CompilationUnit
						&& programCu.getName().equals(superClassName))
					cu.addAncestor(((it.unive.lisa.program.CompilationUnit) programCu));
		}
		if (cu.getImmediateAncestors().isEmpty() && objectUnit != null)
			cu.addAncestor(objectUnit);
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> classInitBody = parseClassBody(ctx.suite());
		classInit.getNodeList().mergeWith(classInitBody.getMiddle());
		NoOp noOp = new NoOp(this.currentCFG, SyntheticLocation.INSTANCE);
		classInit.addNode(noOp, true);
		classInit.addEdge(new SequentialEdge(noOp, classInitBody.getRight()));
		program.addUnit(cu);
		this.currentUnit = previous;
		PyClassType.register(cu.getName(), cu);
		Expression target;
		if (currentUnit instanceof ModuleUnit pmu) {
			target = new PythonUnitAttributeAccessRef(this.currentCFG, getLocation(ctx), pmu,
					new Global(getLocation(ctx), pmu, name, false));
		} else {
			target = new AttributeAccess(this.currentCFG, getLocation(ctx),
					new VariableRef(this.currentCFG, getLocation(ctx), "$self"), name);
		}
		PyAssign classAssign = new PyAssign(this.currentCFG, getLocation(ctx), target,
				new ImportClass(currentCFG, SyntheticLocation.INSTANCE, name, cu));
		block.addNode(classAssign);
		// Ret ret = new Ret(currentCFG, SyntheticLocation.INSTANCE);
		// block.addNode(ret);
		// block.addEdge(new SequentialEdge(classAssign, ret));

		return Triple.of(classAssign, block, classAssign);
	}

	private Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> parseClassBody(
			SuiteContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null;
		Statement last = null;
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
				// parse the function CFG but don't attach yet
				/*
				 * PyCFG funcCFG = visitFuncdef(stmt.compound_stmt().funcdef());
				 * // create FunctionLiteral to hold the CFG FunctionLiteral
				 * funcLiteral = new FunctionLiteral(this.currentCFG,
				 * getLocation(ctx), funcCFG); // assign to the class/module
				 * object AttributeAccess access = new
				 * AttributeAccess(currentCFG, getLocation(stmt), new
				 * VariableRef(currentCFG, getLocation(stmt), "$self",
				 * Untyped.INSTANCE), // or module object
				 * funcCFG.getDescriptor().getName()); PyAssign assign = new
				 * PyAssign(currentCFG, getLocation(ctx), access, funcLiteral);
				 * block.addNode(assign); if (last != null) block.addEdge(new
				 * SequentialEdge(last, assign)); last = assign;
				 */
			} else if (stmt.compound_stmt().async_stmt() != null)
				visitAsync_stmt(stmt.compound_stmt().async_stmt());
			else if (stmt.compound_stmt().decorated() != null)
				visitDecorated(stmt.compound_stmt().decorated());
		}
		Ret ret = new Ret(currentCFG, SyntheticLocation.INSTANCE);
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
		Statement result = nodes.iterator().next();
		return result;

	}
}
