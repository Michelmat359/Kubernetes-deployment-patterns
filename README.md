# Kubernetes ROS 2 Deployment Patterns

This repository provides four deployment patterns for running ROS 2 (Humble) applications on K3s with load simulation and monitoring hooks.

1. **Monolithic Deployment** - simple YAML manifest.
2. **Microservices Deployment** - Helm chart for separated nodes.
tcl4. **Dynamic Module Loading** - Helm chart for runtime composition with FastAPI.

Each pattern includes a workload generator configurable via environment variables or `values.yaml`.
Prometheus and Grafana can be installed via community Helm charts for observability.

## Prerequisites
- K3s cluster
- kubectl
- Helm
- ...

## Deployment Patterns
(TODO: Add some instructions for each pattern)

## Benchmarking metrics
The following metrics are collected for benchmarking:

### Benchmark Metrics Overview

| **Category** | **Metric** | **Symbol** | **Brief Description**                                                     | **Measurement Procedure / Tools**                                                                             |
|---------------|-------------|------------|---------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|
| **Packaging & Distribution** | **Image footprint** | \( S_{\text{img}} \) | Total container image size including all layers (GB).                     | `docker image inspect <image> --format '{{.Size}}'`                                                           |
| **Deployment & Setup** | **Cold installation time** | \( T_{\text{install}} \) | Time from `helm install` command to completion of initial deployment (s). | Timestamp before/after `helm upgrade --install`                                                               |
| **Deployment & Setup** | **App readiness time** | \( T_{\text{ready}} \) | Time between `helm start` and first ROS 2 message received.               | Collected from an add-hoc ROS 2 probe (first message timestamp).                                              |
| **Runtime Performance** | **Inference time (avg)** | \( T_{\text{inf}} \) | Mean end-to-end inference latency per frame (ms).                         | Collected from ROS 2 detection message timestamps (`header.stamp` vs receive time).                           |
| **Runtime Performance** | **Frame rate** | \( f_{\text{FPS}} \) | Number of processed frames per second during steady state.                | Measured by ROS 2 probe (inference timestamps).                                                               |
| **Runtime Performance** | **Jitter** | \( J_{\text{inf}} \) | Variability of inference latency (p95–p50 or std. dev.).                  | Derived from latency distribution captured in ROS 2 probe logs.                                               |
| **Runtime Performance** | **CPU usage** | \( U_{\text{CPU}} \) | Average and max CPU utilization (%) per container.                        | `docker stats` / `kubectl top pods` sampled in `22_runtime_stats.sh`.                                         |
| **Runtime Performance** | **GPU usage** | \( U_{\text{GPU}} \) | Average and max GPU compute utilization (%).                              | `tegrastats` (Jetson).                                      |
| **Runtime Performance** | **I/O load** | \( L_{\text{IO}} \) | Disk read/write throughput during app execution (MB/s).                   | `iostat -xm 1` or `docker stats` I/O columns; aggregated per second.                                          |
| **Energy Efficiency** | **Power consumption** | \( P_{\text{avg}} \) | Average electrical power drawn by device during inference (W).            | `tegrastats`; integrate over runtime.                                  |
| **Energy Efficiency** | **Energy per inference** | \( E_{\text{inf}} \) | Energy cost per processed frame (J/frame).                                | \(E_{\text{inf}} = P_{\text{avg}} \times t_{\text{run}} / N_{\text{frames}}\); derived from power + FPS logs. |
| **Network & Scheduling** | **Network latency** | \( L_{\text{net}} \) | Round-trip latency or bandwidth delay between edge and cloud (ms).        | `ping`, `iperf3`, or `tc qdisc show`; can be injected via `tc netem`.                                         |
| **Network & Scheduling** | **Scheduling latency** | \( T_{\text{sched}} \) | Time from pod creation to container start (s).                            | Extract from `kubectl get events` (`Scheduled`→`Started`) or `kubectl describe pod`.                          |
| **DevOps & Maintainability** | **Config churn** | \( C_{\text{cfg}} \) | Number of changed lines/files per update cycle.                           | `git diff --stat` between consecutive Helm values/config commits.                                             |
| **DevOps & Maintainability** | **CI pipeline time** | \( T_{\text{CI}} \) | Duration of automated build-test-deploy pipeline (min).                   | CI job logs (GitHub Actions / Jenkins / GitLab CI) → total runtime per commit.                                |

---

### Optional Derived Indicators

| **Indicator** | **Formula / Meaning** |
|----------------|-----------------------|
| **Startup efficiency** | \( \eta_{\text{start}} = 1 / (T_{\text{install}} + T_{\text{ready}}) \) — inverse of total time to operational state. |
| **Energy efficiency** | \( \eta_{\text{energy}} = f_{\text{FPS}} / P_{\text{avg}} \) — performance per watt. |
| **Deployment overhead ratio** | \( R_{\text{deploy}} = T_{\text{install}} / T_{\text{ready}} \) — how long setup dominates runtime availability. |
### 
