# ROS 2 Microservices Deployment

This Helm chart deploys ROS 2 nodes as independent microservices communicating via DDS.

## Usage
```
helm install ros2-micro ./helm/ros2-microservices
```

The `values.yaml` allows adjusting the message rate for the load generator.
