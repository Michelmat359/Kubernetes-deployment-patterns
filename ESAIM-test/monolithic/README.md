# Monolithic ROS 2 Deployment

This pattern packages multiple ROS 2 nodes in a single container launched from a single file.

## Usage
```
kubectl apply -f deployment.yaml
```

Adjust the message rate via the `MESSAGE_RATE` environment variable in the deployment.
The `load-simulator` container publishes messages to topic `load_chatter` to emulate workload.

Metrics can be scraped via Prometheus using the Python client if integrated in the `load_sim.py` script.
