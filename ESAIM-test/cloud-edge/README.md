# Cloud-Edge Kubernetes Deployment

This Helm chart deploys ROS 2 components split between cloud and edge nodes.

- **Cloud**: Fleet manager and AI server.
- **Edge**: Sensor and processing nodes.

Node selectors are used to schedule workloads appropriately.

## Usage
```
helm install cloud-edge ./helm/cloud-edge
```

Adjust replicas and message rate via `values.yaml`.
