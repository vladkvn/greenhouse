# Architecture source of truth (diagrams)

This document is the **canonical reference** for how the greenhouse system is structured, how the **edge actuator + telemetry node** behaves, and how **telemetry flows up** and **commands flow down**.

Other documents should **link here** instead of duplicating diagrams. When implementation diverges (topics, payloads, services), either **update this file first** or record an ADR explaining the intentional deviation.

## Scope

- **Whole system**: modular layout (cloud services, broker, database, UI, field nodes).
- **Control loop**: node publishes readings and actuator state; server-side logic computes decisions; commands return to the node.
- **Edge node (first module)**: transport, validation, actuation (servo, pump, light), safety hooks, telemetry/ack.
- **UML views**: component dependencies, sequence for the control loop, state machine for the node runtime.

Concrete **MQTT topic trees** and **JSON schemas** belong in `packages/shared-types` (and mirrored in firmware) once defined; this file defines **structure and responsibilities** they must satisfy.

---

## 1. Modular system architecture

```mermaid
flowchart TB
  subgraph Operator["Operator / automation"]
    Web["apps_web — UI"]
  end

  subgraph Cloud["Server-side (monorepo)"]
    Api["apps_api — REST/WS, config, audit, persistence"]
    Controller["apps_controller — rules, schedules, decisions"]
    Gateway["apps_gateway — MQTT normalization, ingress/egress"]
  end

  subgraph Data["Data and bus"]
    DB[(TimescaleDB)]
    Broker[[Mosquitto MQTT]]
  end

  subgraph Edge["Field modules (one per zone/node)"]
    M1["Actuator + telemetry node #1<br/>servo, pump, light"]
    M2["Actuator + telemetry node #N"]
  end

  subgraph Future["Future modules"]
    ML["services_ml"]
    Vision["services_vision"]
    Sensors["Sensor-only or richer telemetry nodes"]
  end

  Web --> Api
  Api --> DB
  Controller --> Broker
  Gateway --> Broker
  Gateway --> Api
  Broker --> Gateway
  Broker --> Controller

  Broker <-->|"telemetry / commands"| M1
  Broker <-->|"telemetry / commands"| M2

  ML -.-> Api
  Vision -.-> Api
  Sensors -.-> Broker
```

---

## 2. Closed loop: telemetry up, commands down

```mermaid
flowchart LR
  subgraph Edge["Actuator + telemetry node"]
    Sense["Current readings<br/>(actuator state, optional sensors)"]
    Act["Servo / pump / light"]
    Sense -->|"publish telemetry"| Br
    Br -->|"commands"| Act
  end

  subgraph Server["Server"]
    Gw["Gateway — ingest/normalize"]
    Api["API — storage, configuration"]
    Ctrl["Controller — compute from readings"]
    DB[(Database)]
  end

  subgraph UI["Operator"]
    Web["Web UI"]
  end

  Br[[MQTT broker]]

  Sense -->|"telemetry"| Br
  Br --> Gw
  Gw --> Api
  Gw --> Ctrl
  Api --> DB
  Ctrl --> DB
  Web --> Api
  Ctrl -->|"commands"| Br
  Web -.->|"manual intent via API"| Api
  Api -.->|"command publication path"| Ctrl
```

**Intended data flow**

1. The node publishes **telemetry** (measurements, derived actuator state, health).
2. **Gateway** normalizes and may forward to **API** for persistence.
3. **Controller** consumes telemetry (directly or via gateway path), applies rules/schedules, and **publishes commands** to MQTT.
4. The node **subscribes to commands**, executes, and reflects results in subsequent telemetry.

---

## 3. Edge node: internal structure (first module)

```mermaid
flowchart LR
  subgraph ServerPath["Server → command"]
    CtrlS["Controller / API-side publisher"]
    BrIn[[MQTT broker]]
    CtrlS -->|"publish cmd"| BrIn
  end

  subgraph Node["Actuator + telemetry node (e.g. ESP32)"]
    direction TB
    Trans["Transport: Wi-Fi / MQTT client"]
    Parse["Parse + validate command"]
    Router["Route by command type"]
    ExecS["Servo driver"]
    ExecP["Pump driver"]
    ExecL["Light driver"]
    Safety["Guards: timeouts, interlocks, watchdog"]
    State["State + ACK / errors"]
    Trans --> Parse --> Router
    Router --> ExecS
    Router --> ExecP
    Router --> ExecL
    Safety --> ExecS
    Safety --> ExecP
    Safety --> ExecL
    ExecS --> State
    ExecP --> State
    ExecL --> State
    State --> Trans
  end

  BrIn <-->|"subscribe commands<br/>publish telemetry"| Trans
```

**Telemetry composition (conceptual)**

- **Actuator truth**: actual lamp/pump state (and servo position or target), not only last requested command.
- **Optional sensing**: any sensors co-located on the node feed the same telemetry stream when present.
- **Operational**: last accepted command id, errors, firmware/build id as needed for operations.

---

## 4. UML component diagram

Stereotypes use Guillemets-style labels in node text for readability in Mermaid.

```mermaid
flowchart TB
  subgraph ClientTier["client"]
    Web["«component»<br/>Web UI"]
  end

  subgraph ServerTier["server"]
    API["«component»<br/>API"]
    GW["«component»<br/>MQTT Gateway"]
    CTRL["«component»<br/>Controller / rules"]
  end

  subgraph InfraTier["infrastructure"]
    DB[("«database»<br/>TimescaleDB")]
    MQTT[["«queue»<br/>MQTT broker"]]
  end

  subgraph EdgeTier["device"]
    NODE["«component»<br/>Actuator + telemetry node"]
  end

  Web -->|HTTPS| API
  API --> DB
  GW -->|persist / read config| API
  GW <-->|MQTT| MQTT
  CTRL <-->|MQTT| MQTT
  CTRL -->|read policies / history| API
  NODE <-->|telemetry / commands| MQTT
```

---

## 5. UML sequence diagram (readings → compute → command)

```mermaid
sequenceDiagram
  autonumber
  participant N as Actuator node
  participant M as MQTT broker
  participant G as Gateway
  participant C as Controller
  participant A as API
  participant D as Database

  loop Periodic or event-driven
    N->>M: publish telemetry (readings, actuator state)
    M->>G: deliver telemetry
    G->>A: ingest / normalize (as designed)
    A->>D: store time-series / audit
  end

  C->>M: subscribe telemetry (architecture-dependent path)
  C->>C: evaluate rules / compute setpoints
  C->>M: publish command
  M->>N: deliver command
  N->>N: validate + execute (servo / pump / light)
  N->>M: publish telemetry (updated readings / ack)
```

---

## 6. UML state machine (node runtime)

```mermaid
stateDiagram-v2
  [*] --> Offline

  Offline --> Connecting : power / config
  Connecting --> Online : MQTT connected

  state Online {
    [*] --> Idle
    Idle --> Streaming : timer / event
    Streaming --> Idle : telemetry published

    Idle --> Executing : valid command received
    Executing --> Idle : actuation finished / policy timeout
    Executing --> Fault : hardware / driver error
    Streaming --> Fault : publish failure (policy-dependent)
  }

  Online --> Reconnecting : connection lost
  Reconnecting --> Online : restored
  Reconnecting --> Offline : give up / reset

  Fault --> Reconnecting : recoverable
  Fault --> Offline : hard reset
```

---

## Change policy

- **Diagram-first**: update this document when changing boundaries (new service, new node type, new bus).
- **Contracts**: topic names and payload shapes are **not** authoritative here; define them in shared schemas and keep this file aligned at the **box-and-arrow** level.
