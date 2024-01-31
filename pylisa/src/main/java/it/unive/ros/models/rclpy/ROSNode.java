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
import it.unive.lisa.symbolic.heap.HeapExpression;
import it.unive.ros.lisa.analysis.constants.ConstantPropagation;
import it.unive.ros.network.NetworkEntityContainer;
import it.unive.ros.network.NetworkEvent;
import it.unive.ros.permissions.jaxb.*;
import jakarta.xml.bind.JAXBElement;
import java.math.BigInteger;
import java.util.*;
import javax.xml.datatype.DatatypeConfigurationException;
import javax.xml.datatype.DatatypeFactory;

public class ROSNode implements NetworkEntityContainer<ROSNetworkEntity<? extends ROSCommunicationChannel>>{
	private ROSLisaNodeAnalysis lisaNodeAnalysis;

	private String name;
	private String namespace;
	private Set<ROSServiceServer> serviceServers = new HashSet<>();;

	private Set<ROSServiceClient> serviceClients = new HashSet<>();
	private Set<ROSTopicSubscription> subscribers = new HashSet<>();
	private Set<ROSTopicPublisher> publishers = new HashSet<>();

	private Set<ROSActionServer> actionServers = new HashSet<>();

	private Set<ROSActionClient> actionClients = new HashSet<>();

	private ScopeId scopeId;

	private Boolean enableParameterServices = true;

	private Boolean enableRosout = true;
	private Set<ROSNetworkEntity> networkEntities;

	private Set<Action> actions;

	public ROSNode(
			String name,
			ScopeId scopeId) {
		this(name, "", scopeId);
	}

	public ROSNode(String nodeName, String namespace, Boolean startParamService, Boolean enableRosout, Statement node, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, InterproceduralAnalysis<?> interproceduralAnalysis) {
		this(nodeName, namespace, startParamService, node, expr, analysisState, interproceduralAnalysis);
		this.enableRosout = enableRosout;
	}

	private void setNamespace(String namespace) {
		if (!namespace.startsWith("/")) {
			this.namespace = "/" + namespace;
		} else {
			this.namespace = namespace;
		}
	}
	public ROSNode(
			String name,
			String namespace,
			ScopeId scopeId) {
		this.name = name;
		this.scopeId = scopeId;
		setNamespace(namespace);
	}

	public ROSNode(String nodeName, String namespace, Statement node, HeapExpression expr) {
		this.name = nodeName;
		this.scopeId = null;
		setNamespace(namespace);

	}

	public ROSNode(String nodeName, String namespace, Statement node, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, InterproceduralAnalysis<? extends AbstractState<?>> interproceduralAnalysis) {
		this.name = nodeName;
		setNamespace(namespace);
		this.lisaNodeAnalysis = new ROSLisaNodeAnalysis(expr, node, analysisState, interproceduralAnalysis);
	}

	public ROSNode(String nodeName, String namespace, Boolean enableParameterServices, Statement node, HeapExpression expr, AnalysisState<SimpleAbstractState<PointBasedHeap, ValueEnvironment<ConstantPropagation>, TypeEnvironment<InferredTypes>>> analysisState, InterproceduralAnalysis<? extends AbstractState<?>> interproceduralAnalysis) {
		this.enableParameterServices = enableParameterServices;
		this.name = nodeName;
		setNamespace(namespace);
		this.lisaNodeAnalysis = new ROSLisaNodeAnalysis(expr, node, analysisState, interproceduralAnalysis);
	}

	public void addNewSubscriber(
			ROSTopic topic,
			String msgType,
			String callBackFunction) {
		networkEntities.add(new ROSTopicSubscription(this, topic, msgType, null));
	}

	public void addNewService(
			ROSServiceServer service) {
		this.serviceServers.add(service);
	}

	public void addNewAction(Action action) {
		this.actions.add(action);
	}


	public void addNewPublisher(
			ROSTopic topic,
			String msgType) {
		networkEntities.add(new ROSTopicPublisher(this, topic, msgType));
	}


	public Set<ROSTopic> getSubscribersTopics() {
		HashSet<ROSTopic> topics = new HashSet<>();
		for (ROSTopicSubscription s : subscribers) {
			topics.add(s.getChannel());
		}
		return topics;
	}

	public Set<ROSServiceServer> getServiceServers() {
		return this.serviceServers;
	}

	public Set<ROSActionServer> getActionServers() {
		return this.actionServers;
	}

	public Set<ROSActionClient> getActionClients() {
		return this.actionClients;
	}
	public Set<ROSServiceClient> getServiceClients() {
		return this.serviceClients;
	}
	public Set<ROSTopic> getPublisherTopics() {
		HashSet<ROSTopic> topics = new HashSet<>();
		for (ROSTopicPublisher s : publishers) {
			topics.add(s.getChannel());
		}
		return topics;
	}

	public Set<ROSTopic> getAllNodeTopics() {
		Set<ROSTopic> topics = getSubscribersTopics();
		topics.addAll(getPublisherTopics());
		return topics;
	}

	public Set<ROSTopicBasedNetworkEntity> getAllNodeTopicsUsers() {
		Set<ROSTopicBasedNetworkEntity> topics = new HashSet<>(subscribers);
		topics.addAll(publishers);
		return topics;
	}

	public String getName() {
		return name;
	}

	@Override
	public String getURI() {
		if (getNamespace().endsWith("/") || getName().startsWith("/")) {
			return getNamespace() + getName();
		}
		return getNamespace() + "/" + getName();
	}

	public List<NetworkEvent> getProcessedEvents() {
		List<NetworkEvent> result = new ArrayList<>();
		for (ROSNetworkEntity ne : this.getNetworkEntities()) {
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

	public Set<ROSTopicSubscription> getSubscribers() {
		return subscribers;
	}

	public Set<ROSTopicPublisher> getPublishers() {
		return publishers;
	}


	public Set<ROSTopicSubscription> getAllSubscriptions() {
		Set<ROSTopicSubscription> allSubscriptions = new HashSet<>(this.subscribers);
		Set<ROSServiceBasedNetworkEntity> serviceEntities = new HashSet<>(this.serviceServers);
		serviceEntities.addAll(this.serviceClients);
		for (ROSServiceBasedNetworkEntity service: serviceEntities) {
			Set<ROSTopicBasedNetworkEntity> serviceTopics = service.toTopicEntities();
			for (ROSTopicBasedNetworkEntity t : serviceTopics) {
				if (t instanceof ROSTopicSubscription) {
					ROSTopicSubscription s = allSubscriptions.stream()
							.filter(subscription -> t.getChannel().getName().equals(subscription.getChannel().getName()))
							.findAny()
							.orElse(null);
					if (s == null) {
						allSubscriptions.add((ROSTopicSubscription) t);
					}
				}
			}
		}

		Set<ROSActionBasedNetworkEntity> actionEntities = new HashSet<>(this.actionServers);
		actionEntities.addAll(this.actionClients);
		for (ROSActionBasedNetworkEntity action: actionEntities) {
			Set<ROSTopicBasedNetworkEntity> serviceTopics = action.toTopicEntities();
			for (ROSTopicBasedNetworkEntity t : serviceTopics) {
				if (t instanceof ROSTopicSubscription) {
					ROSTopicSubscription s = allSubscriptions.stream()
							.filter(subscription -> t.getChannel().getName().equals(subscription.getChannel().getName()))
							.findAny()
							.orElse(null);
					if (s == null) {
						allSubscriptions.add((ROSTopicSubscription) t);
					}
				}
			}
		}
		return allSubscriptions;
	}

	public Set<ROSTopicPublisher> getAllPublishers() {
		Set<ROSTopicPublisher> allPublishers = new HashSet<>(this.publishers);
		Set<ROSServiceBasedNetworkEntity> serviceEntities = new HashSet<>(this.serviceServers);
		serviceEntities.addAll(this.serviceClients);
		for (ROSServiceBasedNetworkEntity service: serviceEntities) {
			Set<ROSTopicBasedNetworkEntity> serviceTopics = service.toTopicEntities();
			for (ROSTopicBasedNetworkEntity t : serviceTopics) {
				if (t instanceof ROSTopicPublisher) {
					ROSTopicPublisher p = allPublishers.stream()
							.filter(publisher -> t.getChannel().getName().equals(publisher.getChannel().getName()))
							.findAny()
							.orElse(null);
					if (p == null) {
						allPublishers.add((ROSTopicPublisher) t);
					}
				}
			}
		}

		Set<ROSActionBasedNetworkEntity> actionEntities = new HashSet<>(this.actionServers);
		actionEntities.addAll(this.actionClients);
		for (ROSActionBasedNetworkEntity action: actionEntities) {
			Set<ROSTopicBasedNetworkEntity> serviceTopics = action.toTopicEntities();
			for (ROSTopicBasedNetworkEntity t : serviceTopics) {
				if (t instanceof ROSTopicPublisher) {
					ROSTopicPublisher p = allPublishers.stream()
							.filter(publisher -> t.getChannel().getName().equals(publisher.getChannel().getName()))
							.findAny()
							.orElse(null);
					if (p == null) {
						allPublishers.add((ROSTopicPublisher) t);
					}
				}
			}
		}
		return allPublishers;
	}

	public PermissionsNode toPermissionsNode() throws DatatypeConfigurationException {
		/** DDS **/
		PermissionsNode permissionsNode = new PermissionsNode();
		/** [BEGIN] DDS.PERMISSIONS **/
		Permissions permissions = new Permissions();
		/** [BEGIN] DDS.PERMISSIONS.GRANT **/
		Grant grant = new Grant();
		grant.setSubjectName("CN=" + getURI());
		grant.setName(getURI());
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
		for (ROSTopicPublisher p : getAllPublishers()) {
			PtopicExpressionList.getTopic().add(
					p.getChannel().getDDSPrefix() +
							p.getChannel().getName());
		}
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH.TOPICS **/
		Pcriteria.setTopics(PtopicExpressionList);
		/** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH **/
		rule.getPublish().add(Pcriteria);
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE **/
		Criteria Scriteria = new Criteria();
		/** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE.TOPICS **/
		TopicExpressionList StopicExpressionList = new TopicExpressionList();
		for (ROSTopicSubscription s : getAllSubscriptions()) {
			StopicExpressionList.getTopic().add(
					s.getChannel().getDDSPrefix() +
							s.getChannel().getName());
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
				for (ROSTopicPublisher p : this.getPublishers()) {
					boolean found = false;
					for (Criteria c : r.getPublish()) {
						for (String s : c.getTopics().getTopic()) {
							if (p.getChannel().getName().equals(s)) {
								found = true;
							}
						}
					}
					if (!found) {
						res.invalidPublishers.add(p);
					}
				}
				for (ROSTopicSubscription s : this.getSubscribers()) {
					boolean found = false;
					for (Criteria c : r.getSubscribe()) {
						for (String name : c.getTopics().getTopic()) {
							if (s.getChannel().getName().equals(name)) {
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
				for (ROSTopicPublisher p : this.getPublishers()) {
					boolean found = false;
					for (Criteria c : r.getPublish()) {
						for (String s : c.getTopics().getTopic()) {
							if (p.getChannel().getName().equals(s)) {
								found = true;
							}
						}
					}
					if (found) {
						res.invalidPublishers.add(p);
					}
				}
				for (ROSTopicSubscription s : this.getSubscribers()) {
					boolean found = false;
					for (Criteria c : r.getSubscribe()) {
						for (String name : c.getTopics().getTopic()) {
							if (s.getChannel().getName().equals(name)) {
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

	public String getID() {
		return lisaNodeAnalysis.getSymbolicExpression().getCodeLocation().toString();
	}

	public List<ROSNetworkEntity<? extends ROSCommunicationChannel>> getNetworkEntities() {
		List<ROSNetworkEntity<?>> result = new ArrayList<>();
        result.addAll(publishers);
        result.addAll(subscribers);
		result.addAll(serviceClients);
		result.addAll(serviceServers);
		result.addAll(actionServers);
		return result;
	}


	public void addNetworkEntity(ROSNetworkEntity ne) {
		if (ne instanceof ROSTopicPublisher) {
			this.publishers.add((ROSTopicPublisher) ne);
		} else if (ne instanceof ROSTopicSubscription) {
			this.subscribers.add((ROSTopicSubscription) ne);
		} else if (ne instanceof ROSServiceServer) {
			this.serviceServers.add((ROSServiceServer) ne);
		} else if (ne instanceof ROSServiceClient) {
			this.serviceClients.add((ROSServiceClient) ne);
		} else if (ne instanceof ROSActionServer) {
			this.actionServers.add((ROSActionServer) ne);
		} else if (ne instanceof ROSActionClient) {
			this.actionClients.add((ROSActionClient) ne);
		}
	}

	public ROSLisaNodeAnalysis getLisaState() {
		return lisaNodeAnalysis;
	}

	public Boolean getEnableParameterServices() {
		return enableParameterServices;
	}
	public Boolean getEnableRosout() {
		return enableRosout;
	}
}
