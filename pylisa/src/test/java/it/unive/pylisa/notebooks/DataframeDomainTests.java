package it.unive.pylisa.notebooks;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;

public class DataframeDomainTests {

	@Test
	public void simpleTests() {
		DataframeGraphDomain dom = new DataframeGraphDomain();
		
		DataframeGraphDomain top = dom.top();
		assertTrue(top.isTop());
		assertTrue(top.constants.isTop());
		assertTrue(top.constStack.isTop());
		assertTrue(top.graph.isTop());
		assertTrue(top.operations.isTop());
		assertTrue(top.pointers.isTop());
		
		DataframeGraphDomain bot = dom.bottom();
		assertTrue(bot.isBottom());
		assertTrue(bot.constants.isBottom());
		assertTrue(bot.constStack.isBottom());
		assertTrue(bot.graph.isBottom());
		assertTrue(bot.operations.isBottom());
		assertTrue(bot.pointers.isBottom());
	}
}
