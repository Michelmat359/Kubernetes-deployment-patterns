{{/* templates/_helpers.tpl */}}
{{- define "dynamic-loader.name" -}}
{{- default .Chart.Name .Values.nameOverride | trunc 63 | trimSuffix "-" -}}
{{- end }}

{{- define "dynamic-loader.fullname" -}}
{{- if .Values.fullnameOverride }}
  {{- .Values.fullnameOverride | trunc 63 | trimSuffix "-" }}
{{- else }}
  {{- printf "%s-%s" .Release.Name (include "dynamic-loader.name" .) | trunc 63 | trimSuffix "-" }}
{{- end }}
{{- end }}

{{- define "dynamic-loader.labels" -}}
app.kubernetes.io/name: {{ include "dynamic-loader.name" . }}
helm.sh/chart: {{ printf "%s-%s" .Chart.Name .Chart.Version }}
app.kubernetes.io/instance: {{ .Release.Name }}
app.kubernetes.io/version: {{ .Chart.AppVersion }}
app.kubernetes.io/managed-by: {{ .Release.Service }}
{{- end }}
