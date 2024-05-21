# Static analysis of microservices with LiSA #

**This document serves as a collection of research notes and thought processes on the topic of how to step-by-step apply and build a static code analyzer (via LiSA) to microservices.**

**Keywords**: `Static code analyzing`, `LiSA`, `Python`, `Microservices`

### ------ Internship diary page of 12.03.2024

## Rough course of action ##

1. `On what the analyzer is targeting`. Prepare a list of issues, vulnerabilities, and code smells existing in microservices systems.

2. `On how the LiSA analyzer could be extended`. Search and understand how LiSA as a base could be extended. Prepare an importable module that would help and instruct LiSA to work with some main Python microservice frameworks like Fast API, etc. Inspect examples like (lisa4ros2)[ https://github.com/lisa-analyzer/lisa4ros2]. Additionally, investigate how LiSA could consume and scan entire project folders (not just single files) built on the mentioned frameworks.

3. `On the localization of the code of interest`. Frameworks provide a large set of solutions and syntaxis dedicated to the different concerns of services development, build-up, maintenance, and testing. Define a list of syntax examples that are associated with the point of communication. These would be the code portions that LiSA to later statically analyze.

4. `On dissection of syntax`. Learn how to use SARL to decode framework syntax (annotations, declarations, etc.) and make them usable and understandable for further LiSA analysis pipeline.

5. `On results`. Specify what the results of a statically analyzed system of microservices should look like. In the context of issues and warnings investigate how LiSA checkers could output them textually. In the context of visualization investigate how LiSA’s current graph capabilities could accommodate visual representation of microservice provider-consumption edges.

## 1. On what the analyzer is targeting

The specific of static code analysis in the field of microservices is a focus on inter-service issues and code smells, in other words on workflow problematics between two more microservices and their mutual communication or contact points.

### Sources:

#### 1.	*Registries like OWASP, Common Weakness Enumeration (CWE), and NIST National Vulnerability DB.*

`Nuance with the mentioned registries is that there is no dedicated microservice category or tag in their search functionality.` Registries typically focus on individual vulnerabilities and weaknesses rather than systemic issues that arise specifically from the interactions between microservices. Only a bit of extra work and widening some of the individual issues could be transformed into an inter-service one. For example:

**Improper Validation of Specified Type of Input (CWE-1287)**

`Description`: The product receives input that is expected to be of a certain type, but it does not validate or incorrectly validates that the input is actually of the expected type.

`Extension`: From such issues that by nature are solitary, it is **possible to derive and formulate** that in the system of microservices when the object is transferred from one to another their common fields must be in the same type of declaration.

Some more:

**Implicit Conversion (CWE- 690)**

`Description`: The system performs implicit conversion of data types, leading to potential unexpected behavior or vulnerabilities.

`Extension`: Microservices should avoid relying on implicit conversions between data types, as they can lead to unexpected behavior or vulnerabilities during data exchange.

**Incorrect Conversion between Numeric Types (CWE-681)**

`Description`: When converting from one data type to another, such as long to integer, data can be omitted or translated in a way that produces unexpected values. If the resulting values are used in a sensitive context, then dangerous behaviors may occur.

`Extension`: Discourage valid, but implicit numeric conversions like long to integer or double to float between microservices.

**Assignment to Variable without Use (CWE-563)**

`Description`: The variable's value is assigned but never used, making it a dead store.

`Extension`: Microservice provides an object in the response body where certain fields are not used in any other microservice that calls or consumes it.

**Unchecked Return Value (CWE-252)**

`Description`: The variable's value is assigned but never used, making it a dead store.

`Extension`: Remove endpoints that are not used by any other microservice in the system.

**Uncontrolled Resource Consumption (CWE-400)**

`Description`: The system does not properly limit resource consumption, leading to denial of service or performance degradation.

`Extension`: Microservices should implement controls to limit resource consumption, preventing individual services from consuming excessive resources and affecting the overall system performance.

**Exposure of Sensitive Information to an Unauthorized Actor (CWE-200)**

`Description`: The system exposes sensitive information to unauthorized actors, leading to data breaches or privacy violations.

`Extension`: Microservices should delegate which resource is accessible to which microservice in the system.

**Unrestricted File Upload (CWE-434)**

`Description`: The system allows users to upload files without proper validation or enforcement of file types, leading to potential file upload vulnerabilities and execution of malicious code.

`Extension`: Microservices should implement strict file upload validation and enforce restrictions on allowed file types and sizes that are coming from neighboring microservices.

**Insufficient Logging and Monitoring (CWE-798)**

`Description`: The system lacks sufficient logging and monitoring capabilities, hindering detection and response to security incidents or suspicious activities.

`Extension`: Microservice is handling a resource without any logging that comes from neighboring microservice where such logging is present. This loses resource trace path and procedure history.

**Unprotected Transport of Credentials (CWE-523)**

`Description`: Login pages do not use adequate measures to protect the user name and password while they are in transit from the client to the server.

`Extension`: In a microservices architecture, where communication between services is prevalent, a system may establish a network of trusted entities for inter-service communication. However, even within this trusted environment, it remains essential to encode or encrypt credentials during transmission. Implementing secure transport protocols such as HTTPS or utilizing encryption mechanisms ensures that sensitive credentials are adequately protected, mitigating the risk of interception and unauthorized access, thereby maintaining the integrity and security of the system.

### 2.	Trivial issues.

Considerable parts of potential issues are trivial ones. For example:

1.Ensure that both microservices adhere to consistent naming conventions for variables, functions, endpoints, and other elements. E.g. both microservices communicate with identical field names, disallowing any variation like `studentId` from one side and `student_id` from the other.

2.Verify that both microservices use standardized date and time formats across API responses and database interactions. Check if all microservices in the same system have explicit time and date configurations and no time zone misalignment.

3.Ensure that both microservices enforce the Content-Type header for incoming requests, requiring clients to specify the type of data being sent.

4.Verify that operations exposed by both microservices are idempotent when appropriate, meaning that executing the operation multiple times has the same effect as executing it once.

5.Verify that there is no HTTP request misalignment. For example, there is no GET request to the endpoint that accepts POST requests, etc.

6.Check that both microservices handle timeouts gracefully when communicating with other services or resources. Handling timeouts ensures that the system remains responsive and resilient under adverse conditions.

7.Check adherence to standards, like HTTP GET requests should not have a body payload.

8.Check if the requests to the secured endpoints supply authorization parameters.

9.Evaluate that if passed objects or parameters are valid at least at the entry-level (i.e. passes validation specified in the REST controller).

10.Check that endpoint names follow a consistent naming convention across both microservices. Consistent naming conventions improve clarity and ease of understanding, facilitating collaboration among developers and API consumers. An endpoint that is defined all in a small case should not be accessed with the same letters, but in Caps Lock.

11.Handling of Optional Parameters. Verify that both microservices handle optional parameters consistently, including how they interpret and process requests with missing or incomplete parameters. Consistent handling of optional parameters enhances interoperability and prevents unexpected behavior.

12.Ensure that error messages returned by both microservices follow a standardized format and language. Consistent error messages improve clarity and help quickly identify and troubleshoot issues.

### 3.	Researching best practices, known challenges, and potential risks from industry literature.

Researching best practices, known challenges, and potential risks from industry literature.

1.**Dependency Drift**: Lack of proper dependency management and versioning practices can lead to dependency drift, where microservices use different versions of shared libraries or frameworks, potentially introducing compatibility issues and security vulnerabilities.

2.**Lack of Trace Context Propagation**: Inconsistent or inadequate propagation of trace context between microservices can hinder effective distributed tracing, making it challenging to debug and analyze system behavior.

3.**Single Point of Failure**: API gateways can become single points of failure, leading to service disruptions for all microservices behind them.

4.**Distributed Transactions**: Lack of support for distributed transactions can result in inconsistencies across microservices' data stores.

5.**Consistent Authentication Mechanisms**: Ensure that both microservices use consistent authentication mechanisms across endpoints, such as token-based authentication or OAuth, and not both of them.

### 4.	*Investigating issues available in academic sources.*

Academic papers that present their static code analysis tool or solution contain information about issues and code smells that they are resolving. Andrew Walker et al. paper where the static analysis tool MSANose is presented has supplemented with the identification of eleven code smells.

**ESB Usage**: An Enterprise Service Bus (ESB) [2] is a way of message passing between modules of a distributed application in which one module acts as a service bus for all of the other modules to pass messages on. There are pros and cons to this approach. However, in microservices, it can become an issue of creating a single point of failure, and increasing coupling, so it should be avoided.

**Too Many Standards**: Given the distributed nature of the microservice application, multiple discrete teams of developers often work on a given mod]()ule, separate from the other teams. This can create a situation where multiple frameworks are used when a standard should be established for consistency across the modules.

**Wrong Cuts**: This occurs when microservices are split into their technical layers (presentation, business, and data layers). Microservices are supposed to be split by features, and each fully contains their domain’s presentation, business, and data layers.

**Not Having an API Gateway**: The API gateway pattern is a design pattern for managing the connections between microservices. In large, complex systems, this should be used to reduce the potential issues of direct communication.

**Hard-Coded Endpoints**: Hardcoded IP addresses and ports are used to communicate between services. By hardcoding the endpoints, the application becomes more brittle to change and reduces the application’s scalability.

**API Versioning**: All Application Programming Interfaces (API) should be versioned to keep track of changes properly.

**Microservice Greedy**: This occurs when microservices are created for every new feature, and, oftentimes, these new modules are too small and do not serve many purposes. This increases complexity and the overhead of the system. Smaller features should be wrapped into larger microservices if possible.

**Shared Persistency**: When two microservice application modules access the same database, it breaks the microservice definition. Each microservice should have autonomy and control over its data and database.

**Inappropriate Service Intimacy**: One module requesting private data from a separate module also breaks the microservice definition. Each microservice should have control over its private data.

**Shared Libraries**: If microservices are coupled with a common library, that library should be refactored into a separate module. This reduces the fragility of the application by migrating the shared functionality behind a common, unchanging interface. This will make the system resistant to ripples from changes within the library.

**Cyclic Dependency**: This occurs when there is a cyclic connection between calls to different modules. This can cause repetitive calls and also increase the complexity of understanding call traces for developers. This is a poor architectural practice for microservices.

Sebastian Copei et al. [2] demonstrate their IDE plugin SIARest to improve the development of microservice-based systems with static code analysis (Fig 1.).

<img width="1271" alt="img" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/1c9c8158-78c2-4a77-80f4-bcf0ebf99c14">

Fig. 1. Syntax checking with SIARest plugin in Visual Studio Code [2].

From their paper also is possible to derive code smells and issues. In the case of Copei’s paper their reference the GitHub repository where form code one can deduce that their tool is checking [2]:

```js
export const simpleTypeError = (resConf: string, resVal: Expression): SemanticError | undefined => {
    if (resConf === 'string' && resVal.kind !== SyntaxKind.StringLiteral) {
        return createSemanticError('Return value needs to be a string.', resVal.getStart(), resVal.end);
    } else if (resConf === 'number' && resVal.kind !== SyntaxKind.NumericLiteral) {
        return createSemanticError('Return value needs to be a number.', resVal.getStart(), resVal.end);
    } else if (resConf === 'boolean' && resVal.kind !== SyntaxKind.TrueKeyword && resVal.kind !== SyntaxKind.FalseKeyword) {
        return createSemanticError('Return value needs to be true or false.', resVal.getStart(), resVal.end);
    } else if (!['string', 'number', 'boolean'].includes(resConf)) {
        return createSemanticError(`Return value needs to be ${resConf}`, resVal.getStart(), resVal.end);
    }

    return undefined;
};
```

Function to raise error when there is field type differentiation between two microservice endpoints.

Some more grouped by aim:

##### On issues, code smells:

[1]	A. Walker, D. Das, and T. Cerny, “Automated Code-Smell Detection in Microservices Through Static Analysis: A Case StTowards Security-Aware Microservices: On Extracting Endpoint Data Access Operations to Determine Access Rightsudy,” Applied Sciences, vol. 10, no. 21, 2020, doi: 10.3390/app10217800.

[2]	M. and Z. A. Copei Sebastian and Schreiter, “Improving the Implementation of Microservice-Based Systems with Static Code Analysis,” in Agile Processes in Software Engineering and Extreme Programming – Workshops, P. Kruchten Philippe and Gregory, Ed., Cham: Springer Nature Switzerland, 2024, pp. 31–38.

[3]	R. Matar and J. Jahić, “An Approach for Evaluating the Potential Impact of Anti-Patterns on Microservices Performance,” in 2023 IEEE 20th International Conference on Software Architecture Companion (ICSA-C), 2023, pp. 167–170. doi: 10.1109/ICSA-C57050.2023.00044.

##### Discussion:

[4]	T. Černý and D. Taibi, “Microservice-Aware Static Analysis: Opportunities, Gaps, and Advancements,” vol. 111, pp. 2:1-2:14, Jan. 2024, doi: 10.4230/OASIcs.Microservices.2020-2022.2.

#####  Optimization and validation:

[5]	P. Genfer and U. Zdun, “Avoiding Excessive Data Exposure Through Microservice APIs,” in Software Architecture, I. Gerostathopoulos, G. Lewis, T. Batista, and T. Bureš, Eds., Cham: Springer International Publishing, 2022, pp. 3–18.

##### Security:

[6]	A. Abdelfattah., M. Schiewe., J. Curtis., T. Cerny., and E. Song., “Towards Security-Aware Microservices: On Extracting Endpoint Data Access Operations to Determine Access Rights,” in Proceedings of the 13th International Conference on Cloud Computing and Services Science - CLOSER, SciTePress, 2023, pp. 15–23. doi: 10.5220/0011707500003488.

[7]	X. Li, Y. Chen, Z. Lin, X. Wang, and J. H. Chen, “Automatic Policy Generation for Inter-Service Access Control of Microservices,” in 30th USENIX Security Symposium (USENIX Security 21), USENIX Association, Aug. 2021, pp. 3971–3988. [Online]. Available: https://www.usenix.org/conference/usenixsecurity21/presentation/li-xing

##### On how to visualize microservice systems and discovery:

[8]	A. Fekete, B. Kovács, and Z. Porkoláb, “Automatic Dependency Tracking in Microservice-based Systems Using Static Analysis in Helm Charts,” in 2023 International Conference on Software, Telecommunications and Computer Networks (SoftCOM), 2023, pp. 1–7. doi: 10.23919/SoftCOM58365.2023.10271686.

[9]	T. Cerny, A. S. Abdelfattah, V. Bushong, A. Al Maruf, and D. Taibi, “Microservice Architecture Reconstruction and Visualization Techniques: A Review,” in 2022 IEEE International Conference on Service-Oriented System Engineering (SOSE), 2022, pp. 39–48. doi: 10.1109/SOSE55356.2022.00011.

[10]	M. Schiewe, J. Curtis, V. Bushong, and T. Cerny, “Advancing Static Code Analysis With Language-Agnostic Component Identification,” IEEE Access, vol. 10, pp. 30743–30761, 2022, doi: 10.1109/ACCESS.2022.3160485.

[11]	V. Bushong, D. Das, A. Al Maruf, and T. Cerny, “Using Static Analysis to Address Microservice Architecture Reconstruction,” in 2021 36th IEEE/ACM International Conference on Automated Software Engineering (ASE), 2021, pp. 1199–1201. doi: 10.1109/ASE51524.2021.9678749.

[12]	V. Bushong., D. Das., and T. Cerny., “Reconstructing the Holistic Architecture of Microservice Systems using Static Analysis,” in Proceedings of the 12th International Conference on Cloud Computing and Services Science - CLOSER, SciTePress, 2022, pp. 149–157. doi: 10.5220/0011032100003200.

## 3. On the localization of the code of interest

This section for its further examples will consider FastAPI and its specificities in a search and identification of code of interest. There could be multiple ways how microservices could work together, communicating, providing, and making requests.

### Points of communication:

#### 1.	REST Endpoint:

```
 Microservice A                     Microservice B
┌────────────────────┐            ┌──────────────────────────┐
│   /items/{item_id} │───────────►│        getItems()        │
├--------------------┤            │--------------------------│
│   /create_item     │            │--------------------------│
├--------------------┤            │--------------------------│
│   /delete_item     │            │--------------------------│
└────────────────────┘            └──────────────────────────┘
```
At the code side, this could look like this (minimal example):

**Microservice A**

```python
from fastapi import FastAPI, HTTPException

app = FastAPI()

fake_db = {} # In-memory database

@app.get("/items/{item_id}")
async def read_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return {"item_id": item_id, "data": fake_db[item_id]}


@app.post("/items/{item_id}")
async def create_item(item_id: str, data: str):
    if item_id in fake_db:
        raise HTTPException(status_code=400, detail="Item already exists")
    fake_db[item_id] = data
    return {"item_id": item_id, "data": data}


@app.delete("/items/{item_id}")
async def delete_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    del fake_db[item_id]
    return {"message": "Item deleted successfully"}
```

**Microservice B**

```python
def getItems():
    response = requests.get("http://microservice_a:8000/items/1")

    if response.status_code == 200:
        return response.json()
    else:
        return None

items_data = getItems()
print(items_data)
```

#### 2.	Messaging broker:

```
Microservice A              RabbitMQ                Microservice B
┌────────────────┐         ┌─────────────┐         ┌────────────────┐
│ send_message() │────────►│    Queue    │◄────────│receive_messages│
└────────────────┘         └─────────────┘         └────────────────┘
```

At the code side, this could look like this (minimal example):

**Microservice A**

```python
import pika

def send_message(message):
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.queue_declare(queue='messages')
    channel.basic_publish(exchange='', routing_key='messages', body=message)
    print("Sent message:", message)
    connection.close()

message_to_send = "Hello from Microservice A!"
send_message(message_to_send)
```

**Microservice B**

```python
import pika

def receive_messages():
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()
    channel.queue_declare(queue='messages')
    
    def callback(ch, method, properties, body):
        print("Received message:", body.decode())
    
    channel.basic_consume(queue='messages', on_message_callback=callback, auto_ack=True)
    print('Waiting for messages.')
    channel.start_consuming()

receive_messages()
```

Other communcation points to consider: `Shared database`, `GraphQL APIs`, `Service Mesh`, `Remote Procedure Calls (RPC)`, `WebSocket Communication`, `File Systems or Object Storage`, `Event Sourcing and Event Streams`.

## 4. On dissection of syntax

How code of interest is captured in some other academic papers. First one relies on regex-like specification that scans the services code base, but the second one is using a trained NPL to find endpoints and other possible entry points.

1.M. Schiewe, J. Curtis, V. Bushong, and T. Cerny, “Advancing Static Code Analysis With Language-Agnostic Component Identification,” IEEE Access, vol. 10, pp. 30743–30761, 2022, doi: 10.1109/ACCESS.2022.3160485.

<img width="612" alt="spec-to-find" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/d38a7096-9953-4a9f-9ed4-4012b13988db">

2.I. Trabelsi et al., “From legacy to microservices: A type‐based approach for microservices identification using machine learning and semantic analysis,” Journal of Software: Evolution and Process, vol. 35, Mar. 2022, doi: 10.1002/smr.2503.

We propose MicroMiner, a microservice identiﬁcation approach that is based on static-relationship analyses between code elements as well as semantic analyses of the source code. Our approach relies on Machine Learning (ML) techniques and uses service types to guide the identiﬁcation of microservices from legacy monolithic systems.

We adopted a more reﬁned semantic analysis method that uses the pre-trained Word2Vec model based on Google News, which produces more accurate results on the semantic similarity between diﬀerent components of the monolithic project and ensures consistency in the context of microservices.


### ------ Internship diary page of 19.03.2024

### Steps of execution:

1.Take this microservice-a and with the SARL dissect HTTP GET variation into parts for later analysis. Perform syntactic analysis for specified issues.

<table>

<tr>
<td>Endpoint</td> <td>Issue</td>
</tr>

<td>

```python
@app.get()
async def get_status(): 
    return {"status": True}
```
</td>

<td> Ensure that the endpoint string is given </td> 
<tr>

<tr>
<td>

```python
@app.get("/report/{pageNr}")
async def get_report(pageNr: bool):
    return {"report": {}}
```
</td>

<td>
Validate that the input argument is of a type numeric or string type and not any other (dict, boolean, set, list, etc)
</td>

</tr>

<tr>
<td>

```python
@app.get("/items/{itemID}")
def get_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return {"item_id": item_id, "data": fake_db[item_id]}
```
</td>

<td>
Ensure that the path variable in /items/{item_id} is the same as the argument of a function
</td>

</tr>
</table>

Output these issues in a JSON file.

Starting with SARL:

```yaml
library fastapi:
  location fastapi

  method FastAPI: it.unive.pylisa.libraries.fastapi.FastAPI
    libtype fastapi.FastAPI*

  method get: it.unive.pylisa.libraries.fastapi.Get
    libtype fastapi.Operation*
    param path type it.unive.lisa.program.type.StringType::INSTANCE
    param callback type it.unive.pylisa.cfg.type.PyLambdaType::INSTANCE

  method HTTPException: it.unive.pylisa.libraries.fastapi.HTTPException
    libtype fastapi.HTTPException*
    param status_code type it.unive.lisa.program.type.Int32Type::INSTANCE
    param detail type it.unive.lisa.program.type.StringType::INSTANCE

  class fastapi.FastAPI:
    instance method add_route: it.unive.pylisa.libraries.fastapi.AddRoute
      type it.unive.lisa.type.VoidType::INSTANCE
      param self libtype fastapi.FastAPI*
      param path type it.unive.lisa.program.type.StringType::INSTANCE
      param callback type it.unive.pylisa.cfg.type.PyLambdaType::INSTANCE

  class fastapi.Operation:
    instance method execute: it.unive.pylisa.libraries.fastapi.ExecuteOperation
      type it.unive.lisa.type.VoidType::INSTANCE
      param self libtype fastapi.Operation*
      param request type it.unive.lisa.type.Untyped::INSTANCE

  class fastapi.HTTPException:
    instance method respond: it.unive.pylisa.libraries.fastapi.RespondWithHttpException
      type it.unive.lisa.type.VoidType::INSTANCE
      param self libtype fastapi.HTTPException*
```

Creating capture classes. Base `Get`:

<table>
<tr>
<td> Java capturing class </td> <td> SARL </td>
</tr>
<tr>

<td>

```java
public class Get extends NaryExpression implements PluggableStatement {

    private Statement st;

    public Get(CFG cfg, CodeLocation location, Expression... parameters) {
        super(cfg, location, "get", parameters);
    }

    @Override
    public void setOriginatingStatement(Statement st) { this.st = st; }

    @Override
    public Program getProgram() { return super.getProgram(); }

    public static Get build(CFG cfg, CodeLocation location, Expression[] parameters) {
        return new Get(cfg, location, parameters);
    }

    @Override
    protected int compareSameClassAndParams(Statement o) {
        return 0;
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
            InterproceduralAnalysis<A> interproceduralAnalysis,
            AnalysisState<A> analysisState,
            ExpressionSet[] expressionSets,
            StatementStore<A> statementStore) throws SemanticException {
        return null;
    }
}
```
</td>

<td> Get</td>
</tr>
</table>

### ------ Internship diary page of 19.03.2024 - 02.04.2024

#### Problem NR.1 – no matter how much SARL is tinkered some lines of code do not capture. Resolution:

#### Further down below was the done before there was @decorator and type parameter support.

<table>
<tr>
<td> Original </td> <td> Modified </td>
</tr>
<tr>

<td>

```python
@app.get()
async def get_status(): 
    return {"status": True}
```
</td>

<td>

```python
app.get()
def get_status(): 
    return {"status": True}
```
</td>
</tr>
</table>

Still, this was not enough as `def get_status()` is not captured and stays outside the brackets. This could be seen also from the graph:

![Screenshot 2024-03-30 at 22 40 25](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/af6c9266-74e0-4672-b0a4-56c7c18b29b9)

This forced me to level down and create a simplified and minified Python package that could be imported.

```python
# miniapi.py
import re

class MiniAPI:
    def __init__(self):
        self.routes = {}

    def get(self, path):
        # Convert path with placeholders to a regex pattern for matching
        pattern = re.sub(r'{(\w+)}', r'(?P<\1>[^/]+)', path)
        pattern = f'^{pattern}$'

        def decorator(func):
            self.routes[re.compile(pattern)] = func
            return func
        return decorator

    def serve(self, path):
        for pattern, handler in self.routes.items():
            match = pattern.match(path)
            if match:
                # Extract path parameters from the match object
                kwargs = match.groupdict()
                return handler(**kwargs)
        return "404 Not Found"
    
    def response(body, status=200):
        return f"Status: {status}\nBody: {body}"
```

This is how I apply it:

```python
# pseudo_microservice.py

from miniapi import MiniAPI

app = MiniAPI()

@app.get()
def get_status(): 
    return app.response(f"Welcome to MiniAPI Home!", 200)

@app.get("/report/{pageNr}")
def get_report(pageNr: bool):
    return {"report": {}}

@app.get("/items/{itemID}")
def get_item(item_id: str):
    return {"item_id": item_id, "data": {}}

if __name__ == "__main__":
    app.serve("/report/1")
    app.serve("/items/2")
```

This step gave me more visibility of how SARL should approach code and imported dependencies. In `SyntacticCheck` implementation parts like classes, inputs, and arguments appeared and were captured.

![Screenshot 2024-03-30 at 22 50 15](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/4a4c2c7a-81c8-4841-872c-6c8cd038a798)

Apparently as the `def functions` were not executed - LiSA does not treat them as involved in the program – so they are skipped. In the case of running a FastAPI file instance, there is no need for the `main` function or additional trigger as all this is handled by the library itself.

#### Helping myself a little bit more:

```python
app.get()
def get_status(): 
    return {"status": True}

get_status()

app.get("/report/{pageNr}")
def get_report(pageNr: bool):
    return {"report": {}}

get_report(False)

app.get("/items/{itemID}")
def get_item(item_id: str):
    if item_id not in fake_db:
        raise HTTPException(status_code=404, detail="Item not found")
    return {"item_id": item_id, "data": fake_db[item_id]}

get_item("1")
```

The analysis pipeline loops through the captured content in quite a fragmentary way (`visit` functions).

<img width="491" alt="4" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/71ab6083-c040-40aa-96bf-573e175f895f">

So, I need some persistence container.

```java
@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class Endpoint {

    private Method method;
    private String fullPath;
    private String pathVariableName;
    private Param definedMethodVariable;
    private String executedMethodName;
    private Param executedMethodVariable;
    private String lineNr;
```

1.Gathering the `def` functions:

```java
    @Override
public boolean visitUnit(CheckTool tool, Unit unit) {
    for (CodeMember member : unit.getCodeMembers()) {
            ...
```

<img width="1010" alt="1" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/34287d97-3c62-401e-9e72-04bed3850e97">

Getting defined methods names and info about entry arguments:

<img width="378" alt="2" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/6fc2108f-6a62-4fde-8a82-fd0fc28423d7">

#### Problem NR.2 – Where is the type of the `def` functions?

![3](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/43143e45-b482-4104-8bc1-c22ab336fafe)

Going further for the rest of the puzzle:

#### Problem NR.3 – The decorator or its placeholder is detached from the method.

The only place where both are "in the picture" is where the edges are visited:

<img width="575" alt="5" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/1650361a-25f7-4c06-b307-e9ce4870a9b5">

A condition where I know that pair of edges is searched for - they are both unresolved calls.

```java
  @Override
public boolean visit(CheckTool tool, CFG graph, Edge edge) {

    Statement source = edge.getSource();
    Statement destination = edge.getDestination();

    if (source instanceof UnresolvedCall decorator && destination instanceof UnresolvedCall controllerMethod) {
            ...
```

After packing everything together:

<img width="886" alt="6" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/5dce9a09-98fb-4756-950a-054b506ffa0a">

Finally, the analysis part:

```java
    @Override
public void afterExecution(CheckTool tool) {

    for (Endpoint endpoint : endpoints) {

        if (endpoint.getFullPath() == null) {
            tool.warn("No path for endpoint defined at location:" + endpoint.getLineNr());
                ...
```

#### Result "for now":

```json
{
  "warnings" : [ {
    "message" : "[GENERIC] No path for endpoint defined at location:'/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_a.py':7:8"
  }, {
    "message" : "[GENERIC] Path variable name of endpoint '/items/{itemID}' differs from one specified in method. | Path variable name: itemID | Method variable name: item_id | Location: '/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_a.py':19:25"
  }, {
    "message" : "[GENERIC] This endpoint's path method accepts non-string or non-numeric parameter: /report/{pageNr} | Variables type: bool | Location: '/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/actual/microservice_a.py':13:26"
  }]
}
```

### ------ Internship diary page of 09.04.2024

The first thing done was re-writing previous syntactic checks to use decorator and argument-type support.

Now all the "drill-down" could be done in `visitUnit` method:

```java
    @Override
public boolean visitUnit(CheckTool tool, Unit unit) {

    for (CodeMember member : unit.getCodeMembers()) {
        CodeMemberDescriptor descriptor = member.getDescriptor();

        Annotations annotations = descriptor.getAnnotations();
            ...
```

Then adding support for base POST, PUT, and DELETE methods.

Here is the full FastAPI POST method:
```yaml
method post: it.unive.pylisa.libraries.fastapi.httpMethod.Post
  libtype fastapi.Operation*
  param path type it.unive.lisa.program.type.StringType::INSTANCE
  param callback type it.unive.pylisa.cfg.type.PyLambdaType::INSTANCE
  param &response_model type it.unive.lisa.type.Untyped::INSTANCE default none
  param &status_code type it.unive.lisa.program.type.Int32Type::INSTANCE default none
  param &tags type it.unive.lisa.type.Untyped::INSTANCE default none
  param &dependencies type it.unive.lisa.type.Untyped::INSTANCE default none
  param &summary type it.unive.lisa.program.type.StringType::INSTANCE default none
  param &description type it.unive.lisa.program.type.StringType::INSTANCE default none
  param &response_description type it.unive.lisa.program.type.StringType::INSTANCE default "Successful Response"
  param &responses type it.unive.lisa.type.Untyped::INSTANCE default none
  param &deprecated type it.unive.lisa.program.type.BoolType::INSTANCE default none
  param &operation_id type it.unive.lisa.program.type.StringType::INSTANCE default none
  param &response_model_include type it.unive.lisa.type.Untyped::INSTANCE default none
  param &response_model_exclude type it.unive.lisa.type.Untyped::INSTANCE default none
  param &response_model_by_alias type it.unive.lisa.program.type.BoolType::INSTANCE default true
  param &response_model_exclude_unset type it.unive.lisa.program.type.BoolType::INSTANCE default false
  param &response_model_exclude_defaults type it.unive.lisa.program.type.BoolType::INSTANCE default false
  param &response_model_exclude_none type it.unive.lisa.program.type.BoolType::INSTANCE default false
  param &include_in_schema type it.unive.lisa.program.type.BoolType::INSTANCE default true
  param &response_class type it.unive.lisa.type.Untyped::INSTANCE default none
  param &name type it.unive.lisa.program.type.StringType::INSTANCE default none
  param &callbacks type it.unive.lisa.type.Untyped::INSTANCE default none
  param &openapi_extra type it.unive.lisa.type.Untyped::INSTANCE default none
  param &generate_unique_id_function type it.unive.lisa.type.Untyped::INSTANCE default none
```

Trying to move onward to semantic analysis:

<table>
<tr>
<td> From </td> <td> To </td>
</tr>
<tr>

<td>

```java
public class FastApiSyntacticChecker
        implements SyntacticCheck {
``` 
</td>

<td>

```java
public class FastApiChecker
        <A extends AbstractState<A>> implements SemanticCheck<A> {
```
</td>
</tr>
</table>

This may larger listing in the runtime...

![1](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/67d213f4-20ed-49b2-8edf-48ac6237b634)

But there is no "added value" from more elaborate SARL definitions:

![2](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/7a3060d6-618c-4633-9568-dc73a5462ef3)

### Probably there is additional work to do to bring in the value of SARL definitions. The problem is that nothing in these parts is actually run by LiSA:

```java
public class Post extends NaryExpression implements PluggableStatement {

    public Get(CFG cfg, CodeLocation location, Expression... parameters) {}

    @Override
    public void setOriginatingStatement()  {}}

@Override
public Program getProgram() {}

public static Post build(CFG cfg, CodeLocation location, Expression[] parameters) {}

@Override
protected int compareSameClassAndParams(Statement o) { }

@Override
public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux() {}
```

For the time being further elaborations where halted and I moved to involve other HTTP cases.

### Adding additional checks for each other method:

<table>
<tr>
<td> Example </td> <td> Issue </td>
</tr>
<tr>

<td>

```python
class Item():
    name: str
    description: str

@app.post("/item")
def create_item(item: int):
    return {"message": "Item created"}
```
</td>

<td>

Argument or payload must be some kind of object with a defined model. Not a primitive.
</td>
</tr>
</table>

```json
// report.json
{
  "warnings" : {
    "message" : "['/Users/teodors/.../microservice_a.py']: [DESCRIPTOR] This POST endpoint /item does not accept any argument of custom definition / model. Only primitives."
  }
}
```

<table>
<tr>
<td> Example </td> <td> Issue </td>
</tr>
<tr>

<td>

```python
from repository update_item_by_id

@app.put(path = "/item/{item_id}")
def update_item(item_id: str, item: str):
    update_item_by_id(item_id, item)
    return {"message": "Item updated"}
```
</td>

<td>

PUT must have a path variable as ID and custom object as a payload body.
</td>
</tr>
</table>

```json
// report.json
{
  "warnings" : {
    "message" : "['/Users/teodors/.../microservice_a.py']: [DESCRIPTOR] This PUT endpoint /item/{item_id} does not accept any arguments to specify the object for an update."
  }
}
```

<table>
<tr>
<td> Example </td> <td> Issue </td>
</tr>
<tr>

<td>

```python
from repository import get_item_by_id, delete_item_by_id

@app.delete(path = "/item/{item_id}")
def delete_item(item_id: str):
    
    # item = get_item_by_id(item_id)

    # if item is None:
    #     raise HTTPException(status_code=404, detail="Item not found")

    delete_item_by_id(item_id)

    return {"message": "Item delete"}
```
</td>

<td>

The delete method must have some kind of prior item existence checkup or validation
</td>
</tr>
</table>

The criteria to pass the check are twofold:

1. Passed ID argument must be used to retrieve persisted item from DB.
2. That persisted item must be involved in some kind of null check up, that is in `PyIs`.

Additionally, should not be tricked by `improper ordering`:

```python
def delete_item(item_id: str):

    item = get_item_by_id(item_id) # 1
    delete_item_by_id(item_id) # 3
    
    if item is None:
         raise HTTPException(status_code=404, detail="Item not found") # 2
```

And also something like this:

```python
def delete_item(item_id: str):
    
    item = new Item()

    if item is None:
        raise HTTPException(status_code=404, detail="Item not found")

    delete_item_by_id(item)
```

Path of execution:
0. Going down to `visit(Edge edge)`
1. Take entry argument `def delete_item(item_id: str)`.
2. Validate that it is involved in `UnresolvedCall()` that is followed by `PyAssign`.
3. Assigned `VariableRef` must be involved in `PyIs` and compared to the `NullLiteral`.
4. That `VariableRef` should be parented or have some links to the initial `item_id` path argument.

"Helping out" myself and the analyzer (meaning that a slight change is breaking everything) as much as possible, I, of course, got my warning:

```json
// report.json
{
  "warnings" : {
    "message" : "['/Users/teodors/.../microservice_a.py']: [EXPRESSION] DELETE endpoint /item/{item_id} deletes entity without prior check-up for its existence."
  }
}
```

But the hushed problems are obvious:

1. Validation and deletion are usually passed to the service layer and not done in controllers. Meaning that this goes beyond just one file and could span across projects.
2. Showed case covers just one scenario. How about this one:

```python
def delete_item(item_id: str):

    # is_item_present returns bool
    if is_item_present(item_id):
        delete_item_by_id(item)
```

Or...

```python
def delete_item(item_id: str):

    do_delete = lambda item_id: delete_item_by_id(item_id)
    do_nothing = lambda item_id: None  
   
    action = do_delete if is_item_present(item_id) else do_nothing
    action(item_id)
```

3. How could you be sure that `get_item_by_id(item_id)` is a true DB repository function and not some e.g. utility?

4. "Too much helping out"! Adding one other function call breaks the logic:

```python
def delete_item(item_id: str):

    logger("App received new ID to delete from DB")
    item = get_item_by_id(item_id)

    if item is None:
         raise HTTPException(status_code=404, detail="Item not found")

    delete_item_by_id(item_id)
```

5. Absolutely unmaintainable (and unsustainable).

![3](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/d2cfa23c-2905-477f-8c09-84b51b38ad26)

![4](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/53e4601b-954c-46a3-ab89-ec63189febb5)

6. Currently syntactic analysis is almost pure appliance of `spaghetti code` pattern.

Examples:

```java
if (delete != null) {
        if (edge.getSource() instanceof PyIs isCheck) {
        if (entityObtainedFromPathVar) {
        if (!entityPresenceChecked) {
        tool.warnOn(cfg, "...");

```

```java
 if (unit instanceof CodeUnit && !unit.getName().equals("fastapi")) {
        for (CodeMember member : unit.getCodeMembers()) {
        for (Parameter parameter : descriptor.getFormals()) {
        if (endpoints.size() != index) {
```

7. Visiting units, statements, edges dictates laborious handling and control of what is passed further down the analysis and what is excluded.

Here is a "chunky" way of how just `micoservice_a` and `microservice_b` code units are passed further. Avoiding `Set`, `Dict`, etc. class units and other noise.

```java
 if (unit instanceof CodeUnit && !unit.getName().equals("fastapi")) {
```

### At the end of the day.

```json
// report.json
{
  "warnings" : {
    "message" : "[GENERIC] Analysed file/s have | GET: 3 | POST: 1 | PUT: 1| DELETE: 1 endpoints in total."
  }
}
```

### Discussion: There are only unsophisticated and faulty 6 checks present, but the analysis code base is already near 1000 lines!

How to be when there is a plan for 100 checks? How to isolate checks from each other? Something like this:

```java
public class SyntacticCheckerForGET implements SyntacticCheck...
public class SyntacticCheckerForPOST implements SyntacticCheck...
public class SyntacticCheckerForPUT implements SyntacticCheck...
public class SyntacticCheckerForDELETE implements SyntacticCheck...

        conf.syntacticChecks.addAll(new SyntacticCheckerForGET(), new SyntacticCheckerForPOST, new SyntacticCheckerForPUT(),  new SyntacticCheckerForDELETE());
```

### Early involvement of Microservice B...

Microservice B:

```python
import requests

def getItems():
    response = requests.get("http://microservice_a:8000/items/1")

    if response.status_code == 200:
        return response.json()
    else:
        return None
```

First try:

```java
PyFrontend frontend1 = new PyFrontend(
        "/to-analyse/microservice_a.py",
        false);

PyFrontend frontend2 = new PyFrontend(
        "/to-analyse/microservice_b.py",
        false);

Program program1 = frontend1.toLiSAProgram();
Program program2 = frontend2.toLiSAProgram();

LiSA lisa = new LiSA(conf);
lisa.run(program1, program2);
```

But error:

```yaml
Caused by: it.unive.lisa.AnalysisExecutionException: Error while computing fixpoint for entrypoint untyped /to-analyse/microservice_a::$main() [at '/to-analyse/microservice_a.py':1:0]
  at it.unive.lisa.interprocedural.context.ContextBasedAnalysis.processEntrypoints(ContextBasedAnalysis.java:291)
```

Forcing to do what has to be done:

```java
try {
        lisa.run(program1, program2);
} catch (Exception ignored) { }
```

Path of execution:
1. Target the usage of `requests` import.
2. Identify method from function name and path from passed string value.
3. Compare with endpoint collection constructed from microservice_a.

All necessary could be found in `requests.get("http://microservice_a:8000/items/1")`

On `requests`:

<img width="1065" alt="Screenshot 2024-04-10 at 11 38 00" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/311bb9e1-f137-4fac-8770-bafd446b8a0d">

Results:

```json
// report.json
{
  "warnings" : {
    "message" : "[GENERIC] These endpoints are not called even once - GET(/report/{pageNr}), POST(/item), PUT(/item/{item_id}), DELETE(/item/{item_id})"
  }
}
```

Is this a bulletproof analysis?
`lisa.run(program1, program2)` swap to `lisa.run(program2, program1)`

Output:
```
LiSA has encountered an exception while executing the analysis
```

### ------ Internship diary page of 16.04.2024

### Dealing with @decorator problem

1. Move to Django?

```python
# views.py

from django.urls import path
from .views import get_view

urlpatterns = [
    path('get/', get_view),  # URL pattern for the GET request
]
```

```python
# controllers.py

from django.http import HttpResponse

def get_view(request):
    return HttpResponse("Hello, GET request!")
```

2. Move to Pyramid?

```python
from wsgiref.simple_server import make_server
from pyramid.config import Configurator
from pyramid.response import Response

def get_view(request):
    return Response('Hello, GET request!')

if __name__ == '__main__':
    with Configurator() as config:
        config.add_route('get_route', '/get')
        config.add_view(get_view, route_name='get_route', request_method='GET')
        app = config.make_wsgi_app()
    server = make_server('0.0.0.0', 6543, app)
    server.serve_forever()
```

3. Stay with FastApi, but write the code unconventionally

### All three examples are functionally equal

<table>
<tr>
<td> With decorators </td> <td> (1) Without decorators </td> <td> (2) Without decorators </td>
</tr>
<tr>

<td>

```python
@app.get(path = "/report/{pageNr}")
def get_report(pageNr: bool):
    return {"report": {}}
```
</td>

<td>

```python
from fastapi import FastAPI, APIRouter, HTTPException

app = FastAPI()
router = APIRouter()

def get_report(pageNr: bool):
    return {"report": {}}

router.add_api_route(path = "/report/{pageNr}", endpoint=get_report, methods=["GET"])
```
</td>

<td>

```python
from fastapi import FastAPI
app = FastAPI()

def get_report(pageNr: bool):
    return {"report": {}}

get_report = app.get("/report/{pageNr}")(get_report)
```
</td>
</tr>
</table>

But these classes are still not visited...

```java
public class Post extends NaryExpression implements PluggableStatement {

    public Post(CFG cfg, CodeLocation location, Expression... parameters) {}

    @Override
    public void setOriginatingStatement()  {}}

@Override
public Program getProgram() {}

public static Post build(CFG cfg, CodeLocation location, Expression[] parameters) {}

@Override
protected int compareSameClassAndParams(Statement o) { }

@Override
public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux() {}
```

### Building the graph of microservice endpoints

Using (graphviz-java)[https://github.com/nidi3/graphviz-java?tab=readme-ov-file]

After analysis procedure moves to the graph buildup...

```java
    lisa.run(program);

    EndpointGraphBuilder.build(syntacticChecker.endpointsByUnit);
```

What is passed to the graph buildup?

```java
    public class Endpoint {

    private Method method;
    private String fullPath;
    private String pathVariableName;
    private List<Param> methodPathVariable = new ArrayList<>();
    private Role role;

}

public HashMap<String, List<Endpoint>> endpointsByUnit;
```

<img width="530" alt="img1" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/699e9342-241e-4c4d-a03a-1507867369c1">

Which endpoints are providers?

```python
@app.put(path = "/item/{item_id}")
def update_item(item_id: str, item: str):
```

Which endpoints are consumers?

```python
requests.get("http://microservice_a:8000/report/5")
```

#### Finding pairs

1. Spliting endpoints by providers and consumers.
2. Iterating through the providers and in sub-cycle the consumers.

```java
import org.springframework.web.util.UriTemplate;

for (Endpoint provider : providerEndpoints) {
        for (Endpoint consumer : consumerEndpoints) {

String providerPath = provider.getFullPath(); // /item/{item_id}
String consumerPath = consumer.getFullPath(); // /report/5

UriTemplate template = new UriTemplate(providerPath);
Map<String, String> variables = template.match(consumerPath);

            if (!variables.isEmpty()) {

MutableNode providerNode = mutNode(providerLabel);
MutableNode consumerNode = mutNode(consumerLabel);
                    topGraph.add(consumerNode.addLink(providerNode));
```

#### Attempt 1.

![img2](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/b3a1b433-f26e-481d-bfb1-549478b54257)

#### Attempt 2.

![img3](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/12b60ca1-f667-46f2-86cb-879c9ac5008a)

#### Attempt 3.

![img4](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/edf8a68f-a47e-4999-a6b3-5b8dcfec1253)

#### Attempt 4.

All works well if one microservice only consumes, and the other only provides.

![img5](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/01348d43-b54c-4848-9382-601dddf7a597)

#### Attempt 5.

![img6](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/57d260a3-7382-4f05-99ed-ff6e45ac759d)


Where is the problem? Still, an ungracious upload...

```java
PyFrontend frontend1 = new PyFrontend(
        "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/complex/microservice_a.py",
        false);

PyFrontend frontend2 = new PyFrontend(
        "/Users/teodors/Documents/erasmus/lisa/projects/lisa-on-microservices/to-analyse/complex/microservice_b.py",
        false);

Program program1 = frontend1.toLiSAProgram();
Program program2 = frontend2.toLiSAProgram();

LiSA lisa = new LiSA(this.conf);

try {
        lisa.run(program1, program2);
} catch (Exception ignored) {}
```

#### Attempt 6.

![img7](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/51de9347-81a2-43ac-b4c4-f00a64af78bc)

# Opportunities
--------------------------

<img width="1044" alt="img8" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/366780d9-ffed-4d1a-b188-f0c606e3be41">

# And.
--------------------------
<img width="1130" alt="img9" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/31678fef-446e-475b-b7b9-781f2ae033a7">

## To be continued...\.

### To semantic analyisis

```python
from fastapi import FastAPI
app = FastAPI()

# Define the base prefix for the API
API_PREFIX = "/api/v1"

@app.get(path = API_PREFIX + "/users/{user_id}", response_model=User)
def get_user(user_id: int):
    return fake_users_db[user_id]

@app.post(path = API_PREFIX + "/users/", response_model=User)
def create_user(user: User):
    fake_users_db[user.id] = user.dict()
    return user
```

### ------ Internship diary page of 23.04.2024

#### Make multi-file upload more ~~gracious~~  and not sensitive to any ordering.

Much in-depth reconstruction and execution from this:

```java
    Program program1 = frontend1.toLiSAProgram();
Program program2 = frontend2.toLiSAProgram();

LiSA lisa = new LiSA(this.conf);

    try {
            lisa.run(program1, program2);
    } catch (Exception ignored) {}
```

```java
    Program program1 = frontend1.toLiSAProgram();
Program program2 = frontend2.toLiSAProgram();

LiSA lisa = new LiSA(this.conf);
    lisa.run(program1);

LiSA lisa2 = new LiSA(this.conf);
    lisa2.run(program2);
```

#### Involve and execute those `implements PluggableStatement` capture classes.

With Giacomo's help...

<img width="996" alt="img0" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/16944e25-bf0b-4649-9e84-6fe776b13f66">


```python
from fastapi import FastAPI
app = FastAPI()

API_PREFIX = "/api/v1"

@app.get(path = API_PREFIX + "/users/{user_id}", response_model=User)
def get_user(user_id: int):
    return fake_users_db[user_id]
```

#### On improving graph build-up and representation.

Continuing the attempts.

#### Attempt 7.

![img1](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/f3694bcc-0ce3-48f1-b3aa-a62a652ebb41)

#### Attempt 8.

![img2](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/745497ec-3134-461b-95ff-23b74f4c9e9a)

#### Attempt 9.

![img3](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/df4e30f5-68ed-425c-99fe-5f22e8f23d78)

#### Attempt 10.

![img4](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/d4955dbc-5b56-441f-9cc8-58d9f8e74d41)

#### Attempt 11.

![img1](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/53a55995-5f55-4e81-970e-03dba9479974)

More and more information reduces the viewability of the graph. There needs to be some element of interaction (toggling).

### Problem – Neither the vanilla Graphviz nor the Graphviz-java offer an option to export the map in HTML.

<img width="554" alt="img2" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/88e3b514-ecfc-4fef-966b-f258ccd822c8">

But there are javascript libraries that take as input the .dot file contents and render out graphs on the web.

Some options:

### Option 1 - Viz.js

This is a collection of packages for working with <a href="https://graphviz.org">Graphviz</a> in JavaScript. The main package, [viz](./packages/viz), is a WebAssembly build of Graphviz with a simple JavaScript wrapper.

With Viz.js, you can easily render a graph diagram as an SVG element to display it in a webpage:

```js
import { instance } from "@viz-js/viz";

instance().then(viz => {
    document.body.appendChild(viz.renderSVGElement("digraph { a -> b }"))
});
```

### Option 2 - d3-graphviz

Renders SVG from graphs described in the [DOT](https://www.graphviz.org/doc/info/lang.html) language using the [@hpcc-js/wasm](https://hpcc-systems.github.io/hpcc-js-wasm/) port of [Graphviz](http://www.graphviz.org) and does animated transitions between graphs.

## Features
* Rendering of SVG graphs from [DOT](https://www.graphviz.org/doc/info/lang.html) source
* Animated transition of one graph into another
* Edge path tweening
* Node shape tweening
* Fade-in and fade-out of entering and exiting nodes and edges
* Animated growth of entering edges
* Panning & zooming of the generated graph

#### It is based on D3.js

D3 (or D3.js) is a free, open-source JavaScript library for visualizing data. Its low-level approach built on web standards offers unparalleled flexibility in authoring dynamic, data-driven graphics.

The base template:

```html
<!DOCTYPE html>
<html>

<head>
    <title>Base MicroserviceNet graph</title>
    <script src="https://d3js.org/d3.v7.min.js"></script>
    <script src="https://unpkg.com/d3-graphviz/build/d3-graphviz.min.js"></script>
</head>

<body>
<div id="graph"></div>

<script>

    const graphviz = d3.select("#graph").graphviz();

    const dot = `digraph {
      node1 [label="Node 1"];
      node2 [label="Node 2"];
      node1 -> node2;
    }`;

    function renderGraph(dotString) {
        graphviz.renderDot(dotString, function () {});
    }

    renderGraph(dot);

</script>
</body>

</html>
```

#### Attempt 12.

<!DOCTYPE html>
<html>

<head>
  <meta charset="UTF-8">
  <title>Base MicroserviceNet graph</title>

  <script src="https://d3js.org/d3.v7.min.js"></script>
  <script src="https://unpkg.com/d3-graphviz/build/d3-graphviz.min.js"></script>

  <style>
    .hovered {
      fill: red;
      stroke: black;
    }
  </style>
</head>

<body>

  <div id="graph"></div>

  <script>

    const graphviz = d3.select("#graph").graphviz();

    const dot = `digraph "microserviceNET" {
                  graph ["splines"="ortho","bgcolor"="white:white","gradientangle"="90"]
                  subgraph "cluster_microservice_b" {
                  graph ["rankdir"="LR","style"="rounded","color"="black","label"="microservice_b","fontname"="Helvetica-bold","fontsize"="24","splines"="ortho","bgcolor"="grey80:#a1e5a2","gradientangle"="90"]
                  "PUT /manufacturer/{id}" ["color"="lightblue","fontname"="Helvetica","fontsize"="14","style"="filled","label"=<<b>PUT</b><br/>/manufacturer/{id}>]
                  "POST /quota/fik39" ["color"="red3","fontname"="Helvetica","fontsize"="14","fontcolor"="white","style"="filled","label"=<<b>POST</b><br/>/quota/fik39>]
                  "GET /report/3" ["color"="red3","fontname"="Helvetica","fontsize"="14","fontcolor"="white","style"="filled","label"=<<b>GET</b><br/>/report/3>]
                  }
                  subgraph "cluster_microservice_a" {
                  graph ["rankdir"="LR","style"="rounded","color"="black","label"="microservice_a","fontname"="Helvetica-bold","fontsize"="24","splines"="ortho","bgcolor"="grey80:#ecc199","gradientangle"="90"]
                  "GET /report/{pageNr}" ["color"="lightblue","fontname"="Helvetica","fontsize"="14","style"="filled","label"=<<b>GET</b><br/>/report/{pageNr}>]
                  "GET null" ["label"=<<b>GET</b><br/>This endpoint was left without path definition>,"fillcolor"="lightgray","fontname"="Helvetica","fontsize"="14","style"="dotted"]
                  "POST /quota/nau16" ["color"="red3","fontname"="Helvetica","fontsize"="14","fontcolor"="white","style"="filled","label"=<<b>POST</b><br/>/quota/nau16>]
                  "PUT /manufacturer/JoeVendor" ["color"="red3","fontname"="Helvetica","fontsize"="14","fontcolor"="white","style"="filled","label"=<<b>PUT</b><br/>/manufacturer/JoeVendor>]
                  }
                  "PUT /manufacturer/JoeVendor" -> "PUT /manufacturer/{id}"
                  "GET /report/3" -> "GET /report/{pageNr}"
}`;

    function renderGraph(dotString) {
      graphviz.renderDot(dotString, function () {

        d3.selectAll(".node").on("click", function (event, d) {
          const nodeId = d.key;
          alert('Clicked on node: ' + nodeId);
        });

        d3.selectAll(".node")
          .on("mouseover", function () {
            d3.select(this)
              .select("ellipse")
              .classed("hovered", true);
          })
          .on("mouseout", function () {
            d3.select(this)
              .select("ellipse")
              .classed("hovered", false);
          });
      });
    }

    renderGraph(dot);

  </script>
</body>

</html>

#### Task – How can the node pass the non-visible information that could be shown on the on-click event?

In Graphviz specification, no field would be designed to store the arbitrary/additional data. Thus one needs to think about the workarounds.

Two approaches.

1. In a hidden way pass data in the label.

```java
// java

Label.html("<p>This is the visable title</p><p class='hidden'> data1: {endpointName}, data2: {locationOfEndpoint} </p>)
```

```javascript
// html

d3.selectAll(".hidden").style("display", "none"); 
```

But when we export in PNG:
![img1](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/05d9c3be-e50c-44b6-adca-f152167ba1c3)

2. Add custom fields to the node.

```java
// java

providerEndpointNode.add("data", "Additional Info");
```

```dot
// dot

"GET /report/{pageNr}" ["color"="lightblue","fontname"="Helvetica","fontsize"="14","style"="filled","data"="Additional Info","label"=<<b>GET</b><br/>/report/{pageNr}>]
```

```javascript

// html

function renderGraph(dotString) {
    graphviz.renderDot(dotString, function () {})
        .on("end", function () {
            d3.selectAll(".node a").on("click", function () {

                var dataInfo = d3.select(this).attr("xlink:title");

                console.log(d3.select(this));
                // modal.show();
            });
        });
}
```

<img width="664" alt="img3" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/2e4637e2-7821-4b84-997b-1bc21257b4bf">


"Additional Info" should be store in `_groups[0][0].outerHTML`

```html
<ellipse fill="#cd0000" stroke="#cd0000" cx="139" cy="-41.46" rx="123.02" ry="25.46" class=""></ellipse>
<text text-anchor="start" x="125" y="-43.86" font-family="Helvetica,sans-Serif" font-weight="bold" font-size="14.00" fill="white">GET</text>
<text text-anchor="start" x="60.01" y="-29.86" font-family="Helvetica,sans-Serif" font-size="14.00" fill="white">GET /report/{pageNr}</text>
</a>
```

3. Use some rarely needed fields to pass the data...

```java
// java

providerEndpointNode.add("tooltip", "Additional Info");
```

```html
<a xlink:title="Additional Info">
    <ellipse fill="lightblue" stroke="lightblue" cx="207" cy="-97.46" rx="81.76" ry="25.46" class=""></ellipse>
    <text text-anchor="start" x="192.61" y="-99.86" font-family="Helvetica,sans-Serif" font-weight="bold" font-size="14.00">GET</text>
    <text text-anchor="start" x="157.19" y="-85.86" font-family="Helvetica,sans-Serif" font-size="14.00">/report/{pageNr}</text>
</a>
```

### ------ Internship diary page of 30.04.2024

### Reporting potential issue in Lisa multi-file context.

Loading technique:

```java
PyFrontend frontend1 = new PyFrontend("microservice_a.py", false);

PyFrontend frontend2 = new PyFrontend("microservice_b.py", false);

Program program1 = frontend1.toLiSAProgram();
Program program2 = frontend2.toLiSAProgram();

LiSA lisa = new LiSA(this.conf);
lisa.run(program1);

LiSA lisa2 = new LiSA(this.conf);
lisa2.run(program2);
```

Having this in SARL:

```yaml
class fastapi.FastAPI:
  instance method FastAPI: it.unive.pylisa.libraries.fastapi.sarl.FastAPI
    libtype fastapi.FastAPI*
    param self libtype fastapi.FastAPI*
  instance method __init__: it.unive.pylisa.libraries.fastapi.sarl.FastAPI
    libtype fastapi.FastAPI*
    param self libtype fastapi.FastAPI*
```

It enabled to execute `implements PluggableStatement` capture classes.

<img width="996" alt="img0" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/16944e25-bf0b-4649-9e84-6fe776b13f66">


#### But it also had an effect.

Microservice A:

```python
# Provider
from fastapi import FastAPI

app = FastAPI()

@app.get(path = "/items/{itemID}")
def get_item(item_id: str):
    return {"item_id": item_id, "data": fake_db[item_id]}
```

Microservice B:

```python
# Consumer
import requests

def getItems():
    response = requests.get("http://microservice_a:8000/items/1")
```

This should work, but upon executing these two files:

```yaml
Caused by: it.unive.lisa.AnalysisExecutionException: Error while computing fixpoint for entry-point untyped microservice_a::$main() [at '/lisa-on-microservices/to-analyse/actual/microservice_a.py':1:0]

# Later found also:
  Exception while computing semantics of 'app = __init__()'
```

Where is the issue? Microservice B had not defined `app = FastAPI()`.

```python
# Consumer
from fastapi import FastAPI
import requests

app = FastAPI()

def getItems():
    response = requests.get("http://microservice_a:8000/items/1")
```

In this case, Microservice B is purely consumer, but as uses same configuration and the same SARL file, then Microservice B is forced to initate things that it does not actually use.

### ~~Question: How to resolve this? How to make these `__init__()` initiations optional?~~

Resolved as:

```java
    Program program1 = frontend1.toLiSAProgram();
LiSA lisa = new LiSA(this.conf);
    lisa.run(program1);

Program program2 = frontend2.toLiSAProgram();
LiSA lisa2 = new LiSA(this.conf);
    lisa2.run(program2);
```

### Moving to the graph of a web browser.

### Looming problem:

<img width="1262" alt="img2" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/533f2656-cd3c-4b7c-bbd7-76a1c161b4d9">

Taken from:
Černý, Tom & Abdelfattah, Amr & Bushong, Vincent & Maruf, Abdullah & Taibi, Davide. (2022). Microvision: Static analysis-based approach to visualizing microservices in augmented reality. 10.48550/arXiv.2207.02974.

#### My issue would be that the graph template is becoming too packed. The solution would be via Thymelef to deliver sub-views on demand. That would need to host the LiSA project was web web service.

A sketch:

<img width="1317" alt="image" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/49900194-5fb7-4f10-aba3-51e87b01ccd0">

Interacting with the edge:

<img width="1413" alt="img4" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/4b132739-3fcb-4b69-8aa4-6c3bb2da0e49">

Transversing to linked subview:

<img width="1375" alt="img5" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/924a67cc-00d2-45df-ac04-7e94e3134410">

### ------ Internship diary page of 07.05.2024

#### This week consisted of objectives was three-fold:

1. Experiment if Graphviz could render something similar to the sketch, where labeled edges go to the border of a graph and vice versa. Also, edges must be clickable and be able to contain some unvisitable information.
2. Spin PyLiSA instance as Spring Boot Web service accessible via localhost.
3. Introduce the MVC approach in delivering separate graphs sub-views.

A sketch:

<img width="1317" alt="image" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/49900194-5fb7-4f10-aba3-51e87b01ccd0">

### Problem: Connecting something with the empty, invisible, or nonexistent is tricky in graphviz. Some attempts:

![img1](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/a9f7c61e-55b3-428e-b1cf-05a873a750a6)


### Much of a strangeness and anomalies:

![img2](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/701a1619-e52c-4f56-8ca0-f4ee2a30eb3c)

### Much of a strangeness and anomalies:

![img3](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/ac3a7d06-bfda-4cb7-8b59-aad19259a0ea)

![img4](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/dd4d68a4-38b9-44c0-b5f0-73ab27fa7fab)

![img6](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/9469b411-dbfa-4a42-9288-e687e4aac634)


### Then finally:

![dot](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/c14dccbd-5c36-44e1-b1ec-b9e9163a6c92)

### Maybe some better rendering engine???

| Dot       | Osage | Patchwork | Fdp | Sfdp | Twopi | Circo | Neato |
|-----------|-------|-----------|-----|------|-------|-------|-------|
| ![dot](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/17318bda-6909-4e02-a3af-28821eb23ee2) | ![osage](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/47cac9f0-ceb3-40fe-87f0-487bc9e354e9) | ![patchwork](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/3bf3e668-a140-4cc6-bf47-75b6d3bc0bc1) | ![fdp](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/8c654c2c-89e7-4bf2-87a2-16675f38d329) | ![sfdp](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/88eaa398-5bd4-4a31-afee-cf21d99e7b68) | ![twopi](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/93e81666-b8a9-4f62-bc58-6c25d42b9985) | ![circo](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/dd2244d6-d0a2-45a1-913c-8a8839e23737) | ![neato](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/05bea97b-aa55-49c6-a498-3f209b4c3baa)|

### Great, let's add new nodes!

<img width="447" alt="img7" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/7252f077-cd65-4b9f-8847-faa027dda6d6">

<img width="571" alt="img8" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/2b50e293-4184-434f-915a-d543f14cf6b6">

<img width="677" alt="img9" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/b832c4bd-3a1e-4eec-8804-d32052ac0c07">

<img width="519" alt="Screenshot 2024-05-07 at 11 14 53" src="https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/32fcaf19-67cf-488e-8760-8577e04265f9">

### Lowering the ambition.

![img11](https://github.com/TeodorsLisovenko/lisa-on-microservices/assets/45534919/96ed03a4-9e03-4d05-b4d7-6f4a4c5296c3)

## To browser. Demonstration.

### ------ Internship diary page of 21.05.2024 (Final)

## Wrapping up and conclusion

### Preface

*Unfortunately with the PhD proposal little was done to improve aspects of project further. Thus, I only did some cleanup of the code and leaving some elaborations about "what is what"*

#### A Plan for today was...

1. Refactor, refine and wrap up the code. Minor usability improvements for the graph website.
2. Create a summary table that outlines which checks are covered, which needs more testing & development, where core logic is located, etc.

*1. All that was showcased in the last meeting was pushed to separate branch `microservices/springbooted`*

There are currently two versions of project:
`/giacomozanatta/pylisa/tree/mircoservices` - pre-springboot version. One for producing static PNG/HTML/DOT files.
`/giacomozanatta/pylisa/tree/mircoservices/springbooted` - version with springboot version. All unrelated files were removed. Functionality is accessible via localhost:8080.

*2. Summaries*

*Syntactic checks*

| Method | Check                                                                                    | Working Status               | Tested?                 | Refer to class, file or func. name                          |
|--------|------------------------------------------------------------------------------------------|------------------------------|-------------------------|-------------------------------------------------------------|
| ALL    | Endpoint path is given                                                                   | OK                           | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |
| GET    | Input argument is of a type numeric or string type and not any other (dict, boolea, etc) | OK                           | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |                                                           |
| GET    | Path variable name is same as the argument of a function                                 | Unreliable                   | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |                                                           |
| POST   | Argument or payload must be some kind of object with a defined model. Not a primitive.   | Unreliable                   | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |                                                           |
| PUT    | Must have a path variable as ID and a custom object as a payload body.                     | Unreliable                   | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |                                                           |
| DELETE | Must have some kind of prior item existence checkup or validation.                       | Does not work (to be honest) | On undecorated endpoint | FastApiSyntacticChecker + EndpointService + EndpointChecker |                                                          |

*Graph capabilities*

At Java side for exporting to PNG/DOT refer to - [graphviz-java](https://github.com/nidi3/graphviz-java)
For exporting to HTML to - [d3-graphviz](https://github.com/magjac/d3-graphviz)

What is worrying? If you decide to rely on `graphviz-java` notice that its maintenance status is under suspension of abandonment:

![img1](https://github.com/giacomozanatta/pylisa/assets/45534919/1a8b1853-238b-4db2-b9aa-3e23ad173575)

With `d3-graphviz` all looks good!

![img2](https://github.com/giacomozanatta/pylisa/assets/45534919/0ad0fbf0-11d6-4752-8518-0d1d8c19afbf)

If it comes to `interactive data visualizations in web browsers` nothing better than D3.js does not exist. It is the most advanced, customizable, supported in terms of flexibility, web-optimization and performance and community.

Also, for proper, sustainable delivery of complicated views in the browser just Thymeleaf and MVC will not do:

```javascript
    /*<![CDATA[*/
    const dot = /*[[${dotContent}]]*/ 'default DOT placeholder';
    /*]]>*/

    function renderGraph(dotString) {
        graphviz.renderDot(dotString, function () {
            d3.selectAll(".node")
                .on("mouseover", function () {
                    d3.select(this)
                        .select("ellipse")
                        .classed("hovered", true);
                })
                .on("mouseout", function () {
                    d3.select(this)
                        .select("ellipse")
                        .classed("hovered", false);
                })
                .on("click", function () {
                    var label = d3.select(this).select("text").text();
                    console.log(label);
                    console.log(this);
                    if (label) {
                        goToCluster(label);
                    }
                });
        }).on("end", function () {
            d3.selectAll(".node a").each(function () {
```

*Graph capabilities*

| Feature                               | Provided by                            | Refer to class, file or func. name                            |
|---------------------------------------|----------------------------------------|---------------------------------------------------------------|
| Visuals / base representation         | Graphviz                               | GraphController + AnalysisService + GraphServiceForWeb        |
| Render in web browser                 | D3.js                                  | resources/templates/microservice-graph.html                   |
| Page / view construction and delivery | Spring Boot MVC + Thymeleaf            | GraphController                                               |
| Redirect to new views                 | Spring Boot MVC + Thymeleaf + D3.js    | resources/templates/microservice-graph.html + GraphController |
| Dropdown + upload                     | JS + Dropzone +  Spring Boot MVC       | resources/templates/index.html                                | 

All becomes much more maintainable with involvement of frameworks like Angular, React, Vue.js.

*3. Starting point with `springbooted` version. 

The `springbooted` version was run using quite a specific IDEA configuration. Placing here its configuration from `.idea/workspace.xml`

```
  <component name="RunManager">
    <configuration name="PyLiSA" type="SpringBootApplicationConfigurationType" factoryName="Spring Boot" nameIsGenerated="true">
      <option name="FRAME_DEACTIVATION_UPDATE_POLICY" value="UpdateClassesAndResources" />
      <module name="pylisa.main" />
      <option name="SPRING_BOOT_MAIN_CLASS" value="it.unive.pylisa.PyLiSA" />
      <method v="2">
        <option name="Make" enabled="true" />
      </method>
    </configuration>
  </component>
  <component name="SharedIndexes">
    <attachedChunks>
      <set>
        <option value="bundled-jdk-9f38398b9061-39b83d9b5494-intellij.indexing.shared.core-IU-241.15989.150" />
        <option value="bundled-js-predefined-1d06a55b98c1-91d5c284f522-JavaScript-IU-241.15989.150" />
      </set>
    </attachedChunks>
  </component>
  <component name="SpellCheckerSettings" RuntimeDictionaries="0" Folders="0" CustomDictionaries="0" DefaultDictionary="application-level" UseSingleDictionary="true" transferred="true" />
  <component name="TaskManager">
    <task active="true" id="Default" summary="Default task">
      <changelist id="1c11dbb5-3029-4269-85b7-6c177e97982a" name="Changes" comment="" />
      <created>1716237463109</created>
      <option name="number" value="Default" />
      <option name="presentableId" value="Default" />
      <updated>1716237463109</updated>
      <workItem from="1716237464168" duration="25000" />
      <workItem from="1716273176250" duration="115000" />
    </task>
    <servers />
  </component>
  <component name="TypeScriptGeneratedFilesManager">
    <option name="version" value="3" />
  </component>
```

It is best to start experimenting with uploading these files as the analysis/graph build-up is left very fragile.

[microservice-a](https://github.com/giacomozanatta/pylisa/blob/mircoservices/springbooted/pylisa/py-testcases/microservices/level-1-complexity/microservice_a.py)
[microservice-b](https://github.com/giacomozanatta/pylisa/blob/mircoservices/springbooted/pylisa/py-testcases/microservices/level-1-complexity/microservice_b.py)
[microservice-c](https://github.com/giacomozanatta/pylisa/blob/mircoservices/springbooted/pylisa/py-testcases/microservices/level-1-complexity/microservice_c.py)

### Side-notes from researching for PhD proposal...

*1. The industry is slowly diverging away from microservices...*

Some loud titles...

<img width="737" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/93d7174a-ba3c-4291-9bd5-4df4a43d847d">

<img width="662" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/f06366db-7197-47a4-860b-7f58e9cc2538">

<img width="1037" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/76d2bb12-2d57-4520-8a43-5ff9f3942ef4">

<img width="594" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/50494d82-0952-409d-8ce1-6e83279dd734">

<img width="444" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/3c4dc6b4-f4ee-4b37-a8c8-8ef46d36eda7">

### Currently a new architectural pattern is defining. It goes by many names, but is most often referred to as `modular/layered/distributed monolith`

![image](https://github.com/giacomozanatta/pylisa/assets/45534919/ab253f44-343d-466a-8ab0-45cedad5ea18)

*2. Rethinking the fundamentals...*

Having gone through many papers on reconstruction and its possible reliance on static analysis one should examine the "table legs" (t.i. pillars) about e.g. fundamental unit of the MS capture mesh. 

Questions like: Is this optimal and sufficient how for capturing connection points and later doing the reconstruction:

```java
public class Endpoint {

    private Method method;
    private String fullPath;
    private String pathVariableName;
    private String belongs;
    private String codeLocation;
    private Set<String> issues = new HashSet<>();
    private List<Param> methodPathVariable = new ArrayList<>();
    private Role role;
}
```
If additionally consider the involvement of message broker or event-driven based communication - how would look like a universal and probably more abstract capture unit...

Questions like: Not to mention how searching for code of interest and matching is performed. 

Probably the best course of investigation is to build a few minor and interconnect Java projects and test them with the reconstruction tools from the list:
`Appendix to: Tools reconstructing Microservice Architecture: A Systematic Mapping Study` by `Alexander Bakhtin et al.`

Many of the listed tools have Git repositories and are openly examinable:
<img width="873" alt="image" src="https://github.com/giacomozanatta/pylisa/assets/45534919/159aaf56-209d-40d9-aa51-7efb5cb3c5dd">

### Most importantly. I would say LiSA requires more practice-orientated material (or better simple exercise textbook with guided tasks with very dosed theoretical aspects given).

![book](https://github.com/giacomozanatta/pylisa/assets/45534919/71a77cfb-d141-4af2-b528-cfb565a826c4)

When such a book appears. Please email me.

## End.
