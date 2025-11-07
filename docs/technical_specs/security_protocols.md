# 🛡️ MAYURI – Security Protocols & Encryption Standards

**Version:** 1.0  
**Last Updated:** 2025-10-22  
**Author:** MAYURI Robotics Team  

---

## 🔐 Overview

MAYURI’s architecture is **secure-by-design**, ensuring that all communication, data storage, mission control, and AI inference processes are protected from unauthorized access or tampering.  

The system applies **multi-layered security**, combining:
- AES-256 encryption for data transport,  
- RSA-4096 asymmetric keys for authentication,  
- Secure ROS2 DDS communication policies,  
- Role-based access for operator control.

---

## 🧱 Security Architecture Layers

| Layer | Security Mechanism | Description |
|--------|--------------------|-------------|
| **AI Core Layer** | Model Integrity Validation | Verifies model signatures before loading (SHA-256 checksum). |
| **Communication Layer** | AES-256-GCM + RSA Handshake | Ensures encrypted node-to-node messaging. |
| **System Core Layer** | Signed Mission Policies | Validates YAML missions with private key signatures. |
| **Sensor Layer** | Input Integrity Hash | Prevents tampering or spoofing in sensor feeds. |
| **User Interface Layer** | Role-Based Access Control (RBAC) | Grants permissions for control, monitoring, or diagnostics. |
| **Storage Layer** | Encrypted Local Filesystem | All logs, configs, and keys are AES-encrypted at rest. |

---

## 🔒 1. Communication Encryption

All inter-node communications use **AES-256-GCM** (Galois Counter Mode) symmetric encryption.  
Keys are negotiated using an **RSA-4096** asymmetric handshake on session start.

### 🔧 Protocol Steps
1. Nodes exchange **public RSA keys**.
2. A session-specific **AES key** is generated.
3. AES key is encrypted with the peer’s RSA public key.
4. Once both nodes confirm integrity, encrypted communication begins.

### 🔑 Configuration
Defined in:  
`config/security/comm_protocols.yaml`

Example:
```yaml
protocol: "AES-256-GCM"
key_exchange: "RSA-4096"
session_timeout: 3600
integrity_check: true
max_packet_size: 4096
