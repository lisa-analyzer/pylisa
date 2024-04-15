package it.unive.pylisa.libraries.fastapi;

import guru.nidi.graphviz.attribute.Color;
import guru.nidi.graphviz.attribute.Font;
import guru.nidi.graphviz.attribute.Rank;
import guru.nidi.graphviz.attribute.Style;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.Graph;
import it.unive.pylisa.libraries.fastapi.models.Endpoint;
import lombok.experimental.UtilityClass;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static guru.nidi.graphviz.attribute.Attributes.attr;
import static guru.nidi.graphviz.attribute.Rank.RankDir.LEFT_TO_RIGHT;
import static guru.nidi.graphviz.model.Factory.*;

@UtilityClass
public class EndpointGraphBuilder {

    public void build(List<Endpoint> endpoints) throws IOException {

        Graph g = graph("microserviceNET").directed()
                .graphAttr().with(Rank.dir(LEFT_TO_RIGHT))
                .nodeAttr().with(Font.name("arial"))
                .linkAttr().with("class", "link-class")
                .with(
                        node("a").with(Color.RED).link(node("b")),
                        node("b").link(to(node("c")).with(attr("weight", 5), Style.DASHED))
                );

        Graphviz.fromGraph(g).height(100).render(Format.PNG).toFile(new File("example/microserviceNET.png"));
    }
}
