{{- define "cloud-edge.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" -}}
{{- end }}

{{- define "cloud-edge.fullname" -}}
{{- if .Values.fullnameOverride }}
  {{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
  {{- printf "%s-%s" .Release.Name (include "cloud-edge.name" .) | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}

{{- define "cloud-edge.labels" -}}
app.kubernetes.io/name: {{ include "cloud-edge.name" . }}
helm.sh/chart: {{ printf "%s-%s" .Chart.Name .Chart.Version }}
app.kubernetes.io/instance: {{ .Release.Name }}
app.kubernetes.io/version: {{ .Chart.AppVersion }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}
