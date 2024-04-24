package it.unive.pylisa.libraries.fastapi.graph;

import guru.nidi.graphviz.attribute.*;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.*;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.definitions.Role;
import lombok.experimental.UtilityClass;
import org.springframework.web.util.UriTemplate;

import java.io.File;
import java.io.IOException;
import java.util.*;

import static guru.nidi.graphviz.attribute.Color.*;
import static guru.nidi.graphviz.attribute.GraphAttr.SplineMode.ORTHO;
import static guru.nidi.graphviz.attribute.GraphAttr.splines;
import static guru.nidi.graphviz.attribute.Rank.RankDir.LEFT_TO_RIGHT;

import static guru.nidi.graphviz.model.Factory.*;

@UtilityClass
public class EndpointGraphBuilder {

    private final Random rand = new Random();

    private Color randomColor() {
        int r = 128 + rand.nextInt(128);
        int g = 128 + rand.nextInt(128);
        int b = 128 + rand.nextInt(128);
        return Color.rgb(r, g, b);
    }

    public void build(HashMap<String, List<Endpoint>> endpointsByUnit) throws IOException {

        MutableGraph base = mutGraph("microservice_graph").setDirected(true);
        base.graphAttrs().add(splines(ORTHO), WHITE.gradient(WHITE).background().angle(90));

        for (Map.Entry<String, List<Endpoint>> entry : endpointsByUnit.entrySet()) {

            MutableGraph cluster = mutGraph().setDirected(true).setCluster(true).setName(entry.getKey()).graphAttrs().add(Rank.dir(LEFT_TO_RIGHT));
            cluster.graphAttrs().add(Style.ROUNDED);
            cluster.graphAttrs().add(Color.BLACK, Label.of(entry.getKey()));
            cluster.graphAttrs().add(Font.config("Helvetica-bold", 24));
            cluster.graphAttrs().add(splines(ORTHO), GREY80.gradient(randomColor()).background().angle(90));

            List<MutableNode> nodes = new ArrayList<>();
            for (Endpoint endpoint : entry.getValue()) {

                String label = endpoint.getMethod() + " " + endpoint.getFullPath();
                String prettyLabel = "<b> " + endpoint.getMethod() + " </b><br/> " + endpoint.getFullPath();

                if (endpoint.getRole() == Role.CONSUMER) {
                    MutableNode consumer = mutNode(label).setName(label).add(Color.RED3, Font.config("Helvetica", 14), WHITE.font(), Style.FILLED);
                    consumer.add("tooltip", "Additional Info for Node CONSUMER");

                    prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();
                    consumer.add(Label.html(prettyLabel));
                    nodes.add(consumer);

                } else if (endpoint.getFullPath() == null) {
                    MutableNode empty = mutNode(label).setName(label).add(Label.html("<b>" + endpoint.getMethod() + "</b><br/>" + "This endpoint was left without path definition"), Color.LIGHTGRAY.fill(), Font.config("Helvetica", 14), Style.DOTTED);
                    nodes.add(empty);

                } else {
                    MutableNode provider = mutNode(label).setName(label).add(Color.LIGHTBLUE, Font.config("Helvetica", 14), Style.FILLED);

                    prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();
                    provider.add("tooltip", "Additional Info for Node PROVIDER");

                    provider.add(Label.html(prettyLabel));
                    nodes.add(provider);
                }
            }

            cluster.add(nodes);
            base.add(cluster);
        }

        List<Endpoint> consumerEndpoints = endpointsByUnit.values()
                .stream()
                .flatMap(List::stream)
                .filter(endpoint -> endpoint.getRole() == Role.CONSUMER)
                .toList();

        List<Endpoint> providerEndpoints = endpointsByUnit.values()
                .stream()
                .flatMap(List::stream)
                .filter(endpoint -> endpoint.getRole() == Role.PROVIDER)
                .toList();

        for (Endpoint provider : providerEndpoints) {
            for (Endpoint consumer : consumerEndpoints) {

                String providerPath = provider.getFullPath();
                String consumerPath = consumer.getFullPath();
                String providerLabel = provider.getMethod() + " " + providerPath;
                String consumerLabel = consumer.getMethod() + " " + consumerPath;

                if (providerPath != null) {
                    UriTemplate template = new UriTemplate(providerPath);

                    Map<String, String> variables = template.match(consumerPath);

                    if (!variables.isEmpty()) {

                       MutableNode providerNode = mutNode(providerLabel);
                       MutableNode consumerNode = mutNode(consumerLabel);
                       base.add(consumerNode.addLink(providerNode));

                    }
                }
            }
        }

        List<String> connectedConsumers = base.edges()
                .stream()
                .map(edge -> edge.from().name().toString())
                .toList();

        for (Endpoint consumerEndpoint : consumerEndpoints) {

            String consumer = consumerEndpoint.getMethod() + " " + consumerEndpoint.getFullPath();

            if (!connectedConsumers.contains(consumer)) {

                MutableNode from = mutNode(consumer);

                String id = String.valueOf(rand.nextInt());
                MutableNode to = mutNode(" ").setName(id).add(
                        TRANSPARENT.font(),
                        Color.TRANSPARENT,
                        Size.std().margin(0, 0),
                        Image.of("/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/pylisa/pylisa/src/main/resources/assets/cross-mark.png")
                );

                base.add(from.addLink(to));
            }
        }

        Graphviz.fromGraph(base)
                .height(300)
                .render(Format.DOT)
                .toFile(new File("example/microserviceNET.dot"));
    }
}
