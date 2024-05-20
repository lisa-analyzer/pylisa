package it.unive.pylisa.libraries.fastapi.graph;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import guru.nidi.graphviz.attribute.*;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.MutableGraph;
import guru.nidi.graphviz.model.MutableNode;
import it.unive.pylisa.libraries.fastapi.analysis.syntax.EndpointService;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.definitions.GroupBy;
import it.unive.pylisa.libraries.fastapi.definitions.Role;
import org.springframework.stereotype.Service;
import org.springframework.web.util.UriTemplate;

import java.util.*;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import static guru.nidi.graphviz.attribute.Color.*;
import static guru.nidi.graphviz.attribute.GraphAttr.SplineMode.ORTHO;
import static guru.nidi.graphviz.attribute.GraphAttr.splines;
import static guru.nidi.graphviz.attribute.Rank.RankDir.LEFT_TO_RIGHT;
import static guru.nidi.graphviz.model.Factory.mutGraph;
import static guru.nidi.graphviz.model.Factory.mutNode;

// TODO: Much duplication with ordinary GraphService. Better to combine them under one interface.
@Service
public class GraphServiceForWeb {

    private final Random rand = new Random();
    private final ObjectMapper mapper = new ObjectMapper();

    private MutableGraph base;

    private Map<String, List<Endpoint>> grouped;
    private List<Endpoint> providers;
    private List<Endpoint> consumers;

    private final String emptyCallImg = "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/pylisa/pylisa/src/main/resources/static/cross-mark.png";

    private Color randomColor() {
        int r = 128 + rand.nextInt(128);
        int g = 128 + rand.nextInt(128);
        int b = 128 + rand.nextInt(128);
        return Color.rgb(r, g, b);
    }

    public List<String> getClusterOptions(List<Endpoint> endpoints) {
        return EndpointService.groupEndpoints(endpoints, GroupBy.BELONGS).keySet().stream().toList();
    }

    public String buildDot(String microserviceName, List<Endpoint> endpoints) throws IOException {

        this.buildBase(endpoints);

        List<Endpoint> endpointsOfMicro = grouped.get(microserviceName);
        List<MutableNode> nodes = buildNodes(endpointsOfMicro);

        MutableGraph cluster = this.buildClusterBase(microserviceName);
        cluster.add(nodes);

        for (String groupName : grouped.keySet()) {

            if (!groupName.equals(microserviceName)) {
                MutableNode clusterNode = buildClusterAsNode(groupName);
                base.add(clusterNode);
            }
        }

        this.connectNodesToOutgoing(cluster);
        base.add(cluster);

        return Graphviz.fromGraph(base).render(Format.DOT).toString();
    }

    private void buildBase(List<Endpoint> endpoints) {

        this.grouped = EndpointService.groupEndpoints(endpoints, GroupBy.BELONGS);
        this.providers = EndpointService.groupEndpoints(endpoints, GroupBy.ROLE_PROVIDER).values().stream().flatMap(Collection::stream).toList();
        this.consumers = EndpointService.groupEndpoints(endpoints, GroupBy.ROLE_CONSUMER).values().stream().flatMap(Collection::stream).toList();

        MutableGraph base = mutGraph("microservice_graph").setDirected(true);

        base.graphAttrs().add(splines(ORTHO), WHITE.gradient(WHITE).background().angle(90));

        this.base = base;
    }

    private MutableGraph buildClusterBase(String clusterName) {

        MutableGraph cluster = mutGraph().setDirected(true).setCluster(true).setName(clusterName).graphAttrs().add(Rank.dir(LEFT_TO_RIGHT));

        cluster.graphAttrs().add(Style.ROUNDED);
        cluster.graphAttrs().add(Color.BLACK, Label.of(clusterName));
        cluster.graphAttrs().add(Font.config("Helvetica-bold", 24));
        cluster.graphAttrs().add(splines(ORTHO), GREY80.gradient(randomColor()).background().angle(90));

        return cluster;
    }

    private MutableNode buildClusterAsNode(String label) {

        String prettyLabel = "<b>" + label + "</b>";

        MutableNode cluster = mutNode(label).setName(label).add(Label.html(prettyLabel), ROYALBLUE, Font.config("Helvetica", 14), WHITE.font(), Style.FILLED, Shape.RECTANGLE);

        return cluster;
    }

    private MutableNode buildConsumerNode(Endpoint endpoint) throws JsonProcessingException {

        String label = endpoint.getMethod() + " " + endpoint.getFullPath();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();

        MutableNode consumer = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.RED3, Font.config("Helvetica", 14), WHITE.font(), Style.FILLED);

        String json = mapper.writeValueAsString(endpoint);
        consumer.add("tooltip", json);

        return consumer;
    }

    private MutableNode buildProviderNode(Endpoint endpoint) throws JsonProcessingException {

        String label = endpoint.getMethod() + " " + endpoint.getFullPath();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();

        MutableNode provider = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.LIGHTBLUE, Font.config("Helvetica", 14), Style.FILLED);

        String json = mapper.writeValueAsString(endpoint);
        provider.add("tooltip", json);

        return provider;
    }

    private MutableNode buildPathlessNode(Endpoint endpoint) {

        String label = endpoint.getMethod().toString();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + "This endpoint was left without path definition";

        MutableNode empty = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.LIGHTGRAY.fill(), Font.config("Helvetica", 14), Style.DOTTED);

        return empty;
    }

    private List<MutableNode> buildNodes(List<Endpoint> endpoints) throws JsonProcessingException {

        List<MutableNode> nodes = new ArrayList<>();

        for (Endpoint endpoint : endpoints) {

            if (endpoint.getRole() == Role.CONSUMER) {
                MutableNode consumer = buildConsumerNode(endpoint);
                nodes.add(consumer);

            } else if (endpoint.getFullPath() == null) {
                MutableNode empty = buildPathlessNode(endpoint);
                nodes.add(empty);

            } else {
                MutableNode provider = buildProviderNode(endpoint);
                nodes.add(provider);
            }
        }

        return nodes;
    }

    private void connectNodesToOutgoing(MutableGraph cluster) {

        for (Endpoint provider : providers) {
            for (Endpoint consumer : consumers) {

                String providerPath = provider.getFullPath();
                String consumerPath = consumer.getFullPath();

                String providerLabel = provider.getMethod() + " " + providerPath;
                String consumerLabel = consumer.getMethod() + " " + consumerPath;

                if (providerPath != null) {
                    UriTemplate template = new UriTemplate(providerPath);

                    Map<String, String> variables = template.match(consumerPath);

                    if (!variables.isEmpty()) {

                        if (cluster.name().toString().equals(consumer.getBelongs())) {
                            MutableNode consumerNode = mutNode(consumerLabel);
                            MutableNode toCluster = mutNode(provider.getBelongs());
                            base.add(consumerNode.addLink(toCluster));
                        }

                        if (cluster.name().toString().equals(provider.getBelongs())) {
                            MutableNode providerNode = mutNode(providerLabel);
                            MutableNode fromCluster = mutNode(consumer.getBelongs());
                            base.add(fromCluster.addLink(providerNode));
                        }
                    }
                }
            }
        }
    }
}
