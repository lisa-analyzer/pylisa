package it.unive.ros.permissions.jaxb;

import jakarta.xml.bind.*;
import java.io.*;

public class JAXBPermissionsHelpers {

	private static final JAXBContext jaxbContext;

	static {
		try {
			jaxbContext = JAXBContext.newInstance(PermissionsNode.class);
		} catch (JAXBException e) {
			throw new RuntimeException(e);
		}
	}

	static JAXBContext getNewJAXBPermissionsContext() throws JAXBException {
		return jaxbContext;
	}

	public static PermissionsNode load(
			String fileName)
			throws JAXBException,
			FileNotFoundException {
		Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();
		JAXBElement<PermissionsNode> permissions = (JAXBElement<PermissionsNode>) jaxbUnmarshaller
				.unmarshal(new FileInputStream(fileName));
		return permissions.getValue();
	}

	public static void store(
			PermissionsNode permissions,
			String fileName)
			throws JAXBException,
			IOException {
		Marshaller jaxbMarshaller = jaxbContext.createMarshaller();
		jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);
		jaxbMarshaller.setProperty(Marshaller.JAXB_NO_NAMESPACE_SCHEMA_LOCATION,
				"http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd");
		File file = new File(fileName);
		file.getParentFile().mkdirs();
		file.createNewFile();
		jaxbMarshaller.marshal(new ObjectFactory().createDds(permissions), new FileOutputStream(file));
	}

	static void toROSPermissions() {

	}
}
