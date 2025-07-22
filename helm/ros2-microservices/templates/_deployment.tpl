{{/*
Generic deployment template.
Usage: {{ include "ros2-microservices.deployment" (dict "svc" .Values.sensor "name" "sensor") }}
*/}}
{{- define "ros2-microservices.deployment" -}}
apiVersion: apps/v1
kind: Deployment
metadata:
  name: {{ include "ros2-microservices.fullname" $ }}-{{ .name }}
spec:
  replicas: {{ .svc.replicaCount | default 1 }}
  selector:
    matchLabels: { app: {{ include "ros2-microservices.name" $ }}, tier: {{ .name }} }
  template:
    metadata:
      labels: { app: {{ include "ros2-microservices.name" $ }}, tier: {{ .name }} }
    spec:
      containers:
        - name: {{ .name }}
          image: "{{ .svc.image }}:{{ .svc.tag }}"
          command: ["ros2", "launch", "{{ .name }}_bringup", "{{ .name }}.launch.py"]
{{- end }}
