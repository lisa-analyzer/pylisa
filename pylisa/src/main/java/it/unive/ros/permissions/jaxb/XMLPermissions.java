package it.unive.ros.permissions.jaxb;

import jakarta.xml.bind.JAXBElement;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Unmarshaller;

import javax.xml.datatype.DatatypeConfigurationException;
import javax.xml.datatype.DatatypeFactory;
import javax.xml.datatype.Duration;
import javax.xml.datatype.XMLGregorianCalendar;
import javax.xml.namespace.QName;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.*;

public class XMLPermissions {
    /**
     * Returns an instance of XMLPermissions fetching data from an XML File.
     * @param fileName
     * @return
     */
    public static XMLPermissions fromFile(String fileName) throws JAXBException, FileNotFoundException {
        PermissionsNode permissionsXML = JAXBPermissionsHelpers.load(fileName);
        return new XMLPermissions();
    }

    private PermissionsNode toJAXBPermissions() throws DatatypeConfigurationException {
        /** DDS **/
        PermissionsNode permissionsNode = new PermissionsNode();
        /** [BEGIN] DDS.PERMISSIONS **/
        Permissions permissions = new Permissions();
        /** [BEGIN] DDS.PERMISSIONS.GRANT **/
        Grant grant = new Grant();
        grant.setSubjectName("CN=/example");
        grant.setName("example");
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
        PtopicExpressionList.getTopic().add("publish_example_1");
        PtopicExpressionList.getTopic().add("publish_example_2");
        /** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH.TOPICS **/
        Pcriteria.setTopics(PtopicExpressionList);
        /** [END] DDS.PERMISSIONS.GRANT.ALLOW_RULE.PUBLISH **/
        rule.getPublish().add(Pcriteria);
        /** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE **/
        Criteria Scriteria = new Criteria();
        /** [BEGIN] DDS.PERMISSIONS.GRANT.ALLOW_RULE.SUBSCRIBE.TOPICS **/
        TopicExpressionList StopicExpressionList = new TopicExpressionList();
        StopicExpressionList.getTopic().add("subscribe_example_1");
        StopicExpressionList.getTopic().add("subscribe_example_2");
        StopicExpressionList.getTopic().add("subscribe_example_3");
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
    public void toFile(String fileName) throws DatatypeConfigurationException, JAXBException, IOException {

        JAXBPermissionsHelpers.store(toJAXBPermissions(), fileName);
    }
}
