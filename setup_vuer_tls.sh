#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TLS_DIR="${WORKSPACE_ROOT}/.vuer/tls"

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [ubuntu-lan-ip]" >&2
  exit 2
fi

if [[ $# -eq 1 ]]; then
  SERVER_IP="$1"
else
  SERVER_IP="$(ip -4 route get 1.1.1.1 | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}')"
fi

if [[ -z "${SERVER_IP}" || ! "${SERVER_IP}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Could not determine a valid LAN IPv4 address. Pass it explicitly: $0 192.168.1.100" >&2
  exit 1
fi

mkdir -p "${TLS_DIR}"
CERT_FILE="${TLS_DIR}/cert.pem"
KEY_FILE="${TLS_DIR}/key.pem"

if [[ -e "${CERT_FILE}" || -e "${KEY_FILE}" ]]; then
  echo "TLS files already exist in ${TLS_DIR}. Remove them first if the Ubuntu IP has changed." >&2
  exit 1
fi

openssl req \
  -x509 \
  -nodes \
  -newkey rsa:2048 \
  -sha256 \
  -days 825 \
  -keyout "${KEY_FILE}" \
  -out "${CERT_FILE}" \
  -subj "/CN=${SERVER_IP}" \
  -addext "subjectAltName=IP:${SERVER_IP},IP:127.0.0.1,DNS:localhost"
chmod 600 "${KEY_FILE}"

echo
echo "Created Vuer TLS files for ${SERVER_IP}:"
echo "  certificate: ${CERT_FILE}"
echo "  private key: ${KEY_FILE}"
echo
echo "Docker Compose will expose them as /ws/.vuer/tls/cert.pem and key.pem."
echo "Quest URL: https://${SERVER_IP}:8012/?ws=wss://${SERVER_IP}:8012"
