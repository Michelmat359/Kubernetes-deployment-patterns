# Kubernetes ROS 2 Deployment Patterns

This repository provides four deployment patterns for running ROS 2 (Humble) applications on K3s with load simulation and monitoring hooks.

1. **Monolithic Deployment** - simple YAML manifest.
2. **Microservices Deployment** - Helm chart for separated nodes.
3. **Cloud-Edge Deployment** - Helm chart distributing components between cloud and edge nodes.
4. **Dynamic Module Loading** - Helm chart for runtime composition with FastAPI.

Each pattern includes a workload generator configurable via environment variables or `values.yaml`.
Prometheus and Grafana can be installed via community Helm charts for observability.
