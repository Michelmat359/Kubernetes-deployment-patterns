# Dynamic Module Loading

This Helm chart runs a ROS 2 container capable of loading components at runtime.
A simple FastAPI server exposes an HTTP API to load modules dynamically.

## Usage
```
helm install dyn-loader ./helm/dynamic-loader
```

Customize the API port via `values.yaml`.
