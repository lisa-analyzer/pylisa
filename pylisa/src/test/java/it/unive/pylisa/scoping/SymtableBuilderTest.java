package it.unive.pylisa.scoping;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import it.unive.pylisa.ParsingException;
import it.unive.pylisa.antlr.PythonLexer;
import it.unive.pylisa.antlr.PythonParser;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParser.NameContext;
import it.unive.pylisa.scoping.PyScope.ScopeKind;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.junit.Test;

public class SymtableBuilderTest {

	private File_inputContext tree;

	private Symtable build(
			String... lines) {
		String source = String.join("\n", lines) + "\n";
		PythonLexer lexer = new PythonLexer(CharStreams.fromString(source));
		PythonParser parser = new PythonParser(new CommonTokenStream(lexer));
		tree = parser.file_input();
		return SymtableBuilder.build("test.py", "test", tree);
	}

	// asserts that building the symbol table of the source fails
	private void assertSyntaxError(
			String... lines) {
		try {
			build(lines);
		} catch (ParsingException e) {
			return;
		}
		fail("no syntax error reported for:\n" + String.join("\n", lines));
	}

	private static PyScope scope(
			Symtable table,
			String name) {
		PyScope found = null;
		for (PyScope scope : table.getScopes().values())
			if (scope.getName().equals(name)) {
				if (found != null)
					throw new IllegalArgumentException("ambiguous scope name: " + name);
				found = scope;
			}
		assertNotNull("no scope named " + name, found);
		return found;
	}

	// the index-th occurrence (in order of appearance) of the given name in
	// the parse tree
	private NameContext occurrence(
			String name,
			int index) {
		List<NameContext> found = new ArrayList<>();
		collect(tree, name, found);
		return found.get(index);
	}

	private static void collect(
			ParseTree node,
			String name,
			List<NameContext> found) {
		if (node instanceof NameContext && node.getText().equals(name))
			found.add((NameContext) node);
		for (int i = 0; i < node.getChildCount(); i++)
			collect(node.getChild(i), name, found);
	}

	@Test
	public void moduleNamesAreGlobal() {
		Symtable table = build(
				"x = 1",
				"print(x)");
		PyScope module = table.getModuleScope();
		assertEquals(ScopeKind.MODULE, module.getKind());
		assertEquals("test", module.getModule());
		assertEquals(NameKind.GLOBAL_IMPLICIT, module.kindOf("x"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, module.kindOf("print"));
	}

	@Test
	public void functionNamesAreLocalGlobalOrParameters() {
		Symtable table = build(
				"x = 1",
				"g = 2",
				"def f(p, q=x):",
				"    y = p",
				"    global g",
				"    g = y",
				"    return x + q + len(y)");
		PyScope f = scope(table, "f");
		assertEquals(ScopeKind.FUNCTION, f.getKind());
		assertEquals(Set.of("p", "q"), f.getParameters());
		assertEquals(NameKind.LOCAL, f.kindOf("p"));
		assertEquals(NameKind.LOCAL, f.kindOf("q"));
		assertEquals(NameKind.LOCAL, f.kindOf("y"));
		assertEquals(NameKind.GLOBAL_EXPLICIT, f.kindOf("g"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, f.kindOf("x"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, f.kindOf("len"));
		// f is bound in the module
		assertTrue(table.getModuleScope().binds("f"));
		assertFalse(f.binds("x"));
		assertTrue(f.binds("g"));
	}

	@Test
	public void defaultValuesAndAnnotationsAreEvaluatedInTheEnclosingScope() {
		Symtable table = build(
				"def f(a=b, c: T = 0) -> R:",
				"    b = 1",
				"    return b");
		PyScope module = table.getModuleScope();
		PyScope f = scope(table, "f");
		// the occurrence in the default value belongs to the module
		assertSame(module, table.getScopeOf(occurrence("b", 0)));
		assertSame(f, table.getScopeOf(occurrence("b", 1)));
		assertEquals(NameKind.LOCAL, table.kindOf(occurrence("b", 1)));
		assertSame(module, table.getScopeOf(occurrence("T", 0)));
		assertSame(module, table.getScopeOf(occurrence("R", 0)));
	}

	@Test
	public void capturedNamesAreCellsAndFree() {
		Symtable table = build(
				"def outer():",
				"    x = 1",
				"    y = 2",
				"    def middle():",
				"        def inner():",
				"            return x",
				"        return inner",
				"    return middle, y");
		PyScope outer = scope(table, "outer");
		PyScope middle = scope(table, "middle");
		PyScope inner = scope(table, "inner");
		assertEquals(NameKind.CELL, outer.kindOf("x"));
		assertEquals(NameKind.LOCAL, outer.kindOf("y"));
		assertEquals(Set.of("x"), outer.getCellVariables());
		// middle passes x through to inner
		assertEquals(NameKind.FREE, middle.kindOf("x"));
		assertEquals(NameKind.FREE, inner.kindOf("x"));
		assertEquals(Set.of("x"), inner.getFreeVariables());
		assertEquals(NameKind.LOCAL, middle.kindOf("inner"));
	}

	@Test
	public void nonlocalNamesAreFree() {
		Symtable table = build(
				"def counter():",
				"    n = 0",
				"    def inc():",
				"        nonlocal n",
				"        n += 1",
				"        return n",
				"    return inc");
		assertEquals(NameKind.CELL, scope(table, "counter").kindOf("n"));
		assertEquals(NameKind.FREE, scope(table, "inc").kindOf("n"));
		assertTrue(scope(table, "inc").binds("n"));
	}

	@Test
	public void globalInEnclosingFunctionIsNotVisible() {
		Symtable table = build(
				"def f():",
				"    global x",
				"    x = 1",
				"    def g():",
				"        return x");
		assertEquals(NameKind.GLOBAL_EXPLICIT, scope(table, "f").kindOf("x"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, scope(table, "g").kindOf("x"));
	}

	@Test
	public void classBodiesAreNotVisibleFromMethods() {
		Symtable table = build(
				"def f():",
				"    y = 1",
				"    class C:",
				"        x = 1",
				"        y = 2",
				"        def m(self):",
				"            return x, y");
		PyScope c = scope(table, "C");
		PyScope m = scope(table, "m");
		assertEquals(ScopeKind.CLASS, c.getKind());
		assertEquals(NameKind.LOCAL, c.kindOf("x"));
		assertEquals(NameKind.LOCAL, c.kindOf("y"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, m.kindOf("x"));
		// y is the one of f, not the one of the class
		assertEquals(NameKind.FREE, m.kindOf("y"));
		assertEquals(NameKind.CELL, scope(table, "f").kindOf("y"));
	}

	@Test
	public void bindingStatementsBindNames() {
		Symtable table = build(
				"def f(xs):",
				"    import os.path",
				"    import sys as system",
				"    from m import a, b as c",
				"    for i in xs:",
				"        pass",
				"    with open('f') as h:",
				"        pass",
				"    try:",
				"        pass",
				"    except Exception as e:",
				"        pass",
				"    (w := 1)",
				"    t: int",
				"    del xs, z",
				"    class K:",
				"        pass");
		PyScope f = scope(table, "f");
		for (String name : List.of("os", "system", "a", "c", "i", "h", "e", "w", "t", "xs", "z", "K"))
			assertEquals(name, NameKind.LOCAL, f.kindOf(name));
		for (String name : List.of("sys", "b", "path", "open", "Exception", "int"))
			assertFalse(name, f.binds(name));
	}

	@Test
	public void starImportsAreTracked() {
		Symtable table = build("from m import *");
		assertTrue(table.getModuleScope().hasStarImport());
		assertSyntaxError(
				"def f():",
				"    from m import *");
	}

	@Test
	public void comprehensionsOnlyOwnTheirIterationVariables() {
		Symtable table = build(
				"def f(xs, k):",
				"    return [x + k for x in xs if x > (n := 0)]");
		PyScope f = scope(table, "f");
		PyScope comp = scope(table, "<listcomp>");
		assertEquals(ScopeKind.COMPREHENSION, comp.getKind());
		assertEquals(NameKind.LOCAL, comp.kindOf("x"));
		// the other names belong to the function
		assertSame(f, table.getScopeOf(occurrence("k", 1)));
		assertSame(f, table.getScopeOf(occurrence("xs", 1)));
		assertSame(comp, table.getScopeOf(occurrence("x", 0)));
		// the assignment expression binds n in the function
		assertTrue(f.binds("n"));
		assertFalse(comp.binds("n"));
		assertEquals(NameKind.LOCAL, f.kindOf("k"));
	}

	@Test
	public void nestedComprehensionsSeeOuterIterationVariables() {
		Symtable table = build("ys = [[x * y for y in x] for x in xs]");
		PyScope module = table.getModuleScope();
		NameContext xInIterable = occurrence("x", 1);
		NameContext xInElement = occurrence("x", 0);
		PyScope outer = table.getScopeOf(occurrence("x", 2));
		assertEquals(ScopeKind.COMPREHENSION, outer.getKind());
		assertSame(outer, table.getScopeOf(xInIterable));
		assertSame(outer, table.getScopeOf(xInElement));
		assertSame(module, table.getScopeOf(occurrence("xs", 0)));
	}

	@Test
	public void lambdasInsideComprehensionsCaptureIterationVariables() {
		Symtable table = build(
				"def f(xs, k):",
				"    return [lambda: x + k for x in xs]");
		assertEquals(NameKind.CELL, scope(table, "<listcomp>").kindOf("x"));
		assertEquals(NameKind.FREE, scope(table, "<lambda>").kindOf("x"));
		assertEquals(NameKind.FREE, scope(table, "<lambda>").kindOf("k"));
		// k is passed through the comprehension
		assertEquals(NameKind.FREE, scope(table, "<listcomp>").kindOf("k"));
		assertEquals(NameKind.CELL, scope(table, "f").kindOf("k"));
	}

	@Test
	public void patternNamesAreReads() {
		Symtable table = build(
				"def f(c):",
				"    match c:",
				"        case Color.RED:",
				"            return 1",
				"        case Point(x=0):",
				"            return 2",
				"        case other:",
				"            return other");
		PyScope f = scope(table, "f");
		assertSame(f, table.getScopeOf(occurrence("Color", 0)));
		assertEquals(NameKind.GLOBAL_IMPLICIT, table.kindOf(occurrence("Color", 0)));
		assertEquals(NameKind.GLOBAL_IMPLICIT, table.kindOf(occurrence("Point", 0)));
		// keyword patterns and attributes are not reads
		assertNull(table.kindOf(occurrence("RED", 0)));
		assertNull(table.kindOf(occurrence("x", 0)));
		assertEquals(NameKind.LOCAL, f.kindOf("other"));
	}

	@Test
	public void globalsBoundOnlyByModuleCode() {
		assertTrue(build(
				"x = 1",
				"def f():",
				"    global y",
				"    return x + y",
				"class C:",
				"    z = 2").globalsBoundOnlyByModule());
		assertFalse(build(
				"def f():",
				"    global x",
				"    x = 1").globalsBoundOnlyByModule());
		assertFalse(build(
				"def f():",
				"    global x",
				"    del x").globalsBoundOnlyByModule());
		assertFalse(build(
				"def f(s):",
				"    exec(s)").globalsBoundOnlyByModule());
		// exec at module level only affects the code of the module
		assertTrue(build("exec('x = 1')").globalsBoundOnlyByModule());
	}

	@Test
	public void lambdaParametersAreLocal() {
		Symtable table = build(
				"y = 1",
				"f = lambda a, b=y: a + b + y");
		PyScope lambda = scope(table, "<lambda>");
		assertEquals(Set.of("a", "b"), lambda.getParameters());
		assertEquals(NameKind.LOCAL, lambda.kindOf("a"));
		assertEquals(NameKind.GLOBAL_IMPLICIT, lambda.kindOf("y"));
		assertSame(table.getModuleScope(), table.getScopeOf(occurrence("y", 1)));
	}

	@Test
	public void occurrencesInsideNestedFunctionsHaveTheirKind() {
		Symtable table = build(
				"def outer():",
				"    x = 1",
				"    def inner():",
				"        return x",
				"    return x");
		assertEquals(NameKind.CELL, table.kindOf(occurrence("x", 0)));
		assertEquals(NameKind.FREE, table.kindOf(occurrence("x", 1)));
		assertEquals(NameKind.CELL, table.kindOf(occurrence("x", 2)));
		// names that are not read or bound (e.g. attributes) have no kind
		Symtable attributes = build("a.b = 1");
		assertNull(attributes.kindOf(occurrence("b", 0)));
		assertEquals(NameKind.GLOBAL_IMPLICIT, attributes.kindOf(occurrence("a", 0)));
	}

	@Test
	public void scopingErrorsAreReported() {
		List<String[]> wrong = List.of(
				new String[] { "nonlocal x" },
				new String[] { "def f():", "    nonlocal x" },
				new String[] { "x = 1", "def f():", "    nonlocal x" },
				new String[] { "def f(x):", "    global x" },
				new String[] { "def f():", "    x = 1", "    global x" },
				new String[] { "def f():", "    print(x)", "    global x" },
				new String[] { "def f():", "    x = 1", "    def g():", "        global x", "        nonlocal x" },
				new String[] { "def f(a, a):", "    pass" },
				new String[] { "[(x := 1) for x in y]" });
		for (String[] source : wrong)
			assertSyntaxError(source);
	}
}
