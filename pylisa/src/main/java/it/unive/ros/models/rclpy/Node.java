package it.unive.ros.models.rclpy;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.Network;
import it.unive.ros.network.NetworkEntity;
import it.unive.ros.network.NetworkEntityContainer;
import it.unive.ros.network.NetworkEvent;
import it.unive.ros.permissions.jaxb.*;
import jakarta.xml.bind.JAXBElement;
import java.math.BigInteger;
import java.util.*;
import javax.xml.datatype.DatatypeConfigurationException;
import javax.xml.datatype.DatatypeFactory;

public class Node implements NetworkEntityContainer {
	private ROSLisaNodeAnalysis lisaNodeAnalysis;
	private Set<Service> services;
	public InterproceduralAnalysis<? extends AbstractState<?>> interproceduralAnalysis;
	private Set<Client> clients;
	private Set<Subscription> subscribers;
	private Set<Publisher> publishers;
	private String name;
	private ScopeId scopeId;

	private String namespace;


	private Set<Action> actions;

	public Node(
			String name,
			ScopeId scopeId) {
		this(name, "", scopeId);
	}

	public Node(
			String name,
			String namespace,
			ScopeId scopeId) {
		this.name = name;
		subscribers = new HashSet<>();
		publishers = new HashSet<>();
		services = new HashSet<>();
		clients = new HashSet<>();
		actions = new HashSet<>();
		this.scopeId = scopeId;
		this.namespace = namespace;
	}

	public Node(String nodeName, String namespace, Statement node, HeapExpression expr) {
		this.name = nodeName;
		subscribers = new HashSet<>();
		publishers = new HashSet<>();
		services = new HashSet<>();
		clients = new HashSet<>();
		actions = new HashSet<>();
		this.scopeId = null;
		this.namespace = namespace;

	}

	public Node(String nodeName, String namespace, Statement node, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, InterproceduralAnalysis<? extends AbstractState<?>> interproceduralAnalysis) {
		this.name = nodeName;
		subscribers = new HashSet<>();
		publishers = new HashSet<>();
		services = new HashSet<>();
		clients = new HashSet<>();
		actions = new HashSet<>();
		this.namespace = namespace;
		this.lisaNodeAnalysis = new ROSLisaNodeAnalysis(expr, node, analysisState, interproceduralAnalysis);
		var x = 3;
	}

	public void addNewSubscriber(
			Topic topic,
			String msgType,
			String callBackFunction) {
		subscribers.add(new Subscription(this, topic, msgType, null));
	}

	public void addNewService(
			Service service) {
		this.services.add(service);
	}

	public void addNewAction(Action action) {
		this.actions.add(action);
	}

	public void addNewClient(
			Client client) {
		this.clients.add(client);
	}

	public void addNewServiceSubscriber(
			Topic topic,
			String msgType,
			String callBackFunction) {
		subscribers.add(new ServiceSubscription(this, topic, msgType, callBackFunction));
	}

	public void addNewPublisher(
			Topic topic,
			String msgType) {
		publishers.add(new Publisher(this, topic, msgType));
	}

	public void addNewServicePublisher(
			Topic topic,
			String msgType) {
		publishers.add(new ServicePublisher(this, topic, msgType));
	}

	public void removeSubscriber(
			Subscription s) {
		subscribers.remove(s);
	}

	public void removePublisher(
			Publisher p) {
		publishers.remove(p);
	}

	public Set<Topic> getSubscribersTopics() {
		HashSet<Topic> topics = new HashSet<>();
		for (Subscription s : subscribers) {
			topics.add(s.getTopic());
		}
		return topics;
	}

	public Set<Service> getServices() {
		return this.services;
	}

	public Set<Topic> getPublisherTopics() {
		HashSet<Topic> topics = new HashSet<>();
		for (Publisher s : publishers) {
			topics.add(s.getTopic());
		}
		return topics;
	}

	public Set<Topic> getAllNodeTopics() {
		Set<Topic> topics = getSubscribersTopics();
		topics.addAll(getPublisherTopics());
		return topics;
	}

	public Set<TopicUser> getAllNodeTopicsUsers() {
		Set<TopicUser> topics = new HashSet<>(subscribers);
		topics.addAll(publishers);
		return topics;
	}

	public String getName() {
		return name;
	}
	public List<NetworkEvent> getProcessedEvents() {
		List<NetworkEvent> result = new ArrayList<>();
		for (NetworkEntity ne : this.getNetworkEntities()) {
			result.addAll(ne.getProcessedEvents());
		}
		return result;
	}
	public String getNamespace() {
		return namespace;
	}

	public ScopeId getScopeId() {
		return scopeId;
	}

	public Set<Subscription> getSubscribers() {
		return subscribers;
	}

	public Set<Publisher> getPublishers() {
		return publishers;
	}

	public PermissionsNode toPermissionsNode() throws DatatypeConfigurationException {
		/** DDS **/
		PermissionsNode permissionsNode = new PermissionsNode();
		/** [BEGIN] DDS.PERMISSIONS **/
		Permissions permissions = new Permissions();
		/** [BEGIN] DDS.PERMISSIONS.GRANT **/
		Grant grant = new Grant();
		grant.setSubjectName("CN=/" + name);
		grant.setName(name);
		grant.setDefault(DefaultAction.DENY);
		/** [BEGIN] DDS.PERMISSIONS.GRANT.VALIDITY **/
		Validity validity = new Validity();
		GregorianCalendar cal = new GregorianCalendar();
		cal.setTime(new Date());
		validity.setNotBefore(DatatypeFactory.newInstance().newXMLGregorianCalendar(cal));
		cal.add(Calendar.YEAR, 10);
		validity.setNotAfter(DatatypeFactory.newInstance().newXMLGregorianCalendar(cal));
		/** [END] DDS.PERMISSIONS.GRANT.VALIDITY **/
		grant.setValidity(validity);
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE **/
		Rule rule = new Rule();
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.DOMAINS **/
		DomainIdSet domainIdSet = new DomainIdSet();
		domainIdSet.getIdOrIdRange().add(BigInteger.valueOf(0));
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.DOMAINS **/
		rule.setDomains(domainIdSet);
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH **/
		Criteria Pcriteria = new Criteria();
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH.TOPICS **/
		TopicExpressionList PtopicExpressionList = new TopicExpressionList();
		for (Publisher p : publishers) {
			PtopicExpressionList.getTopic().add(
					"/rt" +
							p.getTopic().getName());
		}
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH.TOPICS **/
		Pcriteria.setTopics(PtopicExpressionList);
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH **/
		rule.getPublish().add(Pcriteria);
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE **/
		Criteria Scriteria = new Criteria();
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE.TOPICS **/
		TopicExpressionList StopicExpressionList = new TopicExpressionList();
		for (Subscription s : subscribers) {
			StopicExpressionList.getTopic().add(
					"/rt" +
							s.getTopic().getName());
		}
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE.TOPICS **/
		Scriteria.setTopics(StopicExpressionList);
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE **/
		rule.getSubscribe().add(Scriteria);
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE **/
		grant.getAllowRuleOrDenyRule().add(new ObjectFactory().createGrantAllowRule(rule));
		/** [END] DDS.PERMISSIONS.GRANT **/
		permissions.getGrant().add(grant);
		/** [END] DDS.PERMISSIONS **/
		permissionsNode.setPermissions(permissions);
		/** [END] DDS **/
		return permissionsNode;
	}

	public PermissionSatisfabilityNodeResult satisfiesPermission(
			PermissionsNode permissions) {
		PermissionSatisfabilityNodeResult res = new PermissionSatisfabilityNodeResult(this);
		Permissions perm = permissions.getPermissions();
		Grant grant = perm.getGrant().get(0); // TODO: iterate over Grant, check
		// grant name.
		// We assume that:
		// 1. there is at least one Grant
		// 2. the first Grant contains the rules for this node.
		// default action is DENY.
		// 1. Check all rules
		for (JAXBElement<Rule> rule : grant.getAllowRuleOrDenyRule()) {
			if (rule.getName().equals("allow_rule") && grant.getDefault().equals(DefaultAction.DENY)) {
				Rule r = rule.getValue();
				for (Publisher p : this.getPublishers()) {
					boolean found = false;
					for (Criteria c : r.getPublish()) {
						for (String s : c.getTopics().getTopic()) {
							if (p.getTopic().getName().equals(s)) {
								found = true;
							}
						}
					}
					if (!found) {
						res.invalidPublishers.add(p);
					}
				}
				for (Subscription s : this.getSubscribers()) {
					boolean found = false;
					for (Criteria c : r.getSubscribe()) {
						for (String name : c.getTopics().getTopic()) {
							if (s.getTopic().getName().equals(name)) {
								found = true;
							}
						}
					}
					if (!found) {
						res.invalidSubscriptions.add(s);
					}
				}
			} else if (rule.getName().equals("deny_rule")) {
				Rule r = rule.getValue();
				for (Publisher p : this.getPublishers()) {
					boolean found = false;
					for (Criteria c : r.getPublish()) {
						for (String s : c.getTopics().getTopic()) {
							if (p.getTopic().getName().equals(s)) {
								found = true;
							}
						}
					}
					if (found) {
						res.invalidPublishers.add(p);
					}
				}
				for (Subscription s : this.getSubscribers()) {
					boolean found = false;
					for (Criteria c : r.getSubscribe()) {
						for (String name : c.getTopics().getTopic()) {
							if (s.getTopic().getName().equals(name)) {
								found = true;
							}
						}
					}
					if (found) {
						res.invalidSubscriptions.add(s);
					}
				}
			}
		}
		return res;
	}

	public void dumpPermissions(
			String workDir)
			throws Exception {
		JAXBPermissionsHelpers.store(this.toPermissionsNode(),
				workDir + "/permissions/" + this.getName() + "/permissions.xml");
	}

	public String dumpPermissions()
			throws Exception {
		return JAXBPermissionsHelpers.toString(this.toPermissionsNode());
	}

	public Set<Action> getActions() {
		return actions;
	}

	@Override
	public String getID() {
		return lisaNodeAnalysis.getSymbolicExpression().getCodeLocation().toString();
	}

	@Override
	public List<NetworkEntity> getNetworkEntities() {
		List<NetworkEntity> result = new ArrayList<>();
		result.addAll(this.publishers);
		result.addAll(this.subscribers);
		return result;
	}

	@Override
	public void addNetworkEntity(NetworkEntity ne) {
		if (ne instanceof Publisher) {
			this.publishers.add((Publisher) ne);
		} else if (ne instanceof Subscription) {
			this.subscribers.add((Subscription) ne);
		}
	}

	public ROSLisaNodeAnalysis getLisaState() {
		return lisaNodeAnalysis;
	}

}
