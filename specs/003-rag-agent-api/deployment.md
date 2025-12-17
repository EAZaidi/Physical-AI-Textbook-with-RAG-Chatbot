# Deployment Guide: RAG Agent API

Production deployment guide for the RAG Agent API with Docker, Kubernetes, and monitoring setup.

## Table of Contents

1. [Docker Deployment](#docker-deployment)
2. [Kubernetes Deployment](#kubernetes-deployment)
3. [Environment-Specific Configurations](#environment-specific-configurations)
4. [Monitoring & Observability](#monitoring--observability)
5. [API Key Management & Rotation](#api-key-management--rotation)
6. [Scaling & Performance](#scaling--performance)
7. [Troubleshooting](#troubleshooting)

---

## Docker Deployment

### Build Docker Image

```bash
# From project root
cd backend

# Build image
docker build -t rag-agent-api:latest -f Dockerfile .

# Tag for registry
docker tag rag-agent-api:latest your-registry.com/rag-agent-api:v1.0.0

# Push to registry
docker push your-registry.com/rag-agent-api:v1.0.0
```

### Run with Docker Compose

The `docker-compose.yml` includes PostgreSQL and Qdrant services:

```bash
# Start all services
docker-compose up -d

# View logs
docker-compose logs -f agent-api

# Stop services
docker-compose down
```

### Standalone Docker Run

```bash
docker run -d \
  --name rag-agent-api \
  --env-file .env \
  -p 8000:8000 \
  --restart unless-stopped \
  rag-agent-api:latest
```

### Docker Compose Production Configuration

```yaml
# docker-compose.prod.yml
version: '3.8'

services:
  agent-api:
    image: your-registry.com/rag-agent-api:v1.0.0
    container_name: rag-agent-api
    restart: always
    ports:
      - "8000:8000"
    env_file:
      - .env.production
    environment:
      - LOG_LEVEL=WARNING
      - LOG_JSON=true
    depends_on:
      - postgres
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8000/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 4G
        reservations:
          cpus: '1'
          memory: 2G

  postgres:
    image: postgres:16-alpine
    container_name: agent-postgres
    restart: always
    environment:
      POSTGRES_USER: agent_user
      POSTGRES_PASSWORD: ${DB_PASSWORD}
      POSTGRES_DB: agent_db
    volumes:
      - postgres_data:/var/lib/postgresql/data
    ports:
      - "5432:5432"
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U agent_user"]
      interval: 10s
      timeout: 5s
      retries: 5

volumes:
  postgres_data:
    driver: local
```

---

## Kubernetes Deployment

### Namespace

```yaml
# k8s/namespace.yaml
apiVersion: v1
kind: Namespace
metadata:
  name: rag-agent-api
  labels:
    app: rag-agent-api
    environment: production
```

### Secrets

```yaml
# k8s/secrets.yaml
apiVersion: v1
kind: Secret
metadata:
  name: agent-api-secrets
  namespace: rag-agent-api
type: Opaque
stringData:
  OPENAI_API_KEY: "sk-..."
  QDRANT_API_KEY: "..."
  DATABASE_URL: "postgresql://agent_user:password@postgres-service:5432/agent_db"
  API_KEY: "your-production-api-key"
```

Apply secrets:
```bash
kubectl apply -f k8s/secrets.yaml
```

### ConfigMap

```yaml
# k8s/configmap.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: agent-api-config
  namespace: rag-agent-api
data:
  OPENAI_MODEL: "gpt-4o-mini"
  QDRANT_URL: "https://prod-qdrant.cloud.qdrant.io"
  QDRANT_COLLECTION: "textbook_chunks"
  LOG_LEVEL: "WARNING"
  LOG_JSON: "true"
  API_RATE_LIMIT: "100"
  MAX_QUERY_LENGTH: "1000"
  TOP_K_DEFAULT: "5"
  SIMILARITY_THRESHOLD_DEFAULT: "0.7"
  CONFIDENCE_THRESHOLD_HIGH: "0.85"
  CONFIDENCE_THRESHOLD_MEDIUM: "0.75"
  CONFIDENCE_THRESHOLD_LOW: "0.60"
  SESSION_EXPIRY_HOURS: "2"
  DB_POOL_SIZE: "50"
  DB_MAX_OVERFLOW: "100"
```

### Deployment

```yaml
# k8s/deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: rag-agent-api
  namespace: rag-agent-api
  labels:
    app: rag-agent-api
    version: v1.0.0
spec:
  replicas: 3
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
  selector:
    matchLabels:
      app: rag-agent-api
  template:
    metadata:
      labels:
        app: rag-agent-api
        version: v1.0.0
    spec:
      containers:
      - name: agent-api
        image: your-registry.com/rag-agent-api:v1.0.0
        ports:
        - containerPort: 8000
          name: http
          protocol: TCP
        envFrom:
        - configMapRef:
            name: agent-api-config
        - secretRef:
            name: agent-api-secrets
        resources:
          requests:
            cpu: 500m
            memory: 1Gi
          limits:
            cpu: 2000m
            memory: 4Gi
        livenessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 30
          periodSeconds: 30
          timeoutSeconds: 10
          failureThreshold: 3
        readinessProbe:
          httpGet:
            path: /health
            port: 8000
          initialDelaySeconds: 10
          periodSeconds: 10
          timeoutSeconds: 5
          failureThreshold: 3
      restartPolicy: Always
```

### Service

```yaml
# k8s/service.yaml
apiVersion: v1
kind: Service
metadata:
  name: rag-agent-api-service
  namespace: rag-agent-api
  labels:
    app: rag-agent-api
spec:
  type: ClusterIP
  ports:
  - port: 80
    targetPort: 8000
    protocol: TCP
    name: http
  selector:
    app: rag-agent-api
```

### Ingress

```yaml
# k8s/ingress.yaml
apiVersion: networking.k8s.io/v1
kind: Ingress
metadata:
  name: rag-agent-api-ingress
  namespace: rag-agent-api
  annotations:
    cert-manager.io/cluster-issuer: "letsencrypt-prod"
    nginx.ingress.kubernetes.io/ssl-redirect: "true"
    nginx.ingress.kubernetes.io/rate-limit: "100"
spec:
  ingressClassName: nginx
  tls:
  - hosts:
    - api.example.com
    secretName: rag-agent-api-tls
  rules:
  - host: api.example.com
    http:
      paths:
      - path: /
        pathType: Prefix
        backend:
          service:
            name: rag-agent-api-service
            port:
              number: 80
```

### Horizontal Pod Autoscaler

```yaml
# k8s/hpa.yaml
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: rag-agent-api-hpa
  namespace: rag-agent-api
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: rag-agent-api
  minReplicas: 3
  maxReplicas: 10
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
  behavior:
    scaleDown:
      stabilizationWindowSeconds: 300
      policies:
      - type: Percent
        value: 50
        periodSeconds: 60
    scaleUp:
      stabilizationWindowSeconds: 60
      policies:
      - type: Percent
        value: 100
        periodSeconds: 30
```

### Deploy to Kubernetes

```bash
# Apply all manifests
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/secrets.yaml
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/deployment.yaml
kubectl apply -f k8s/service.yaml
kubectl apply -f k8s/ingress.yaml
kubectl apply -f k8s/hpa.yaml

# Check deployment status
kubectl get pods -n rag-agent-api
kubectl get svc -n rag-agent-api
kubectl get ing -n rag-agent-api

# View logs
kubectl logs -f deployment/rag-agent-api -n rag-agent-api

# Scale manually
kubectl scale deployment/rag-agent-api --replicas=5 -n rag-agent-api
```

---

## Environment-Specific Configurations

### Development

**Characteristics:**
- Local development
- Debug logging enabled
- Higher rate limits
- Single replica

**Configuration:**
```env
LOG_LEVEL=DEBUG
API_RATE_LIMIT=1000
DB_POOL_SIZE=10
CONFIDENCE_THRESHOLD_LOW=0.55  # More lenient for testing
```

### Staging

**Characteristics:**
- Pre-production testing
- Stricter validation
- Production-like setup
- 2-3 replicas

**Configuration:**
```env
LOG_LEVEL=INFO
LOG_JSON=true
API_RATE_LIMIT=100
DB_POOL_SIZE=30
CONFIDENCE_THRESHOLD_LOW=0.65  # Stricter for validation
```

### Production

**Characteristics:**
- High availability (3+ replicas)
- Optimized for performance
- Comprehensive monitoring
- Auto-scaling enabled

**Configuration:**
```env
LOG_LEVEL=WARNING
LOG_JSON=true
LOG_FILE=/var/log/agent_api/app.log
API_RATE_LIMIT=100
DB_POOL_SIZE=50
DB_MAX_OVERFLOW=100
SESSION_EXPIRY_HOURS=2
CONFIDENCE_THRESHOLD_LOW=0.60
```

---

## Monitoring & Observability

### Health Check Endpoint

```bash
# Check API health
curl https://api.example.com/health

# Response:
{
  "status": "healthy",
  "dependencies": {
    "qdrant": {"status": "up", "latency_ms": 45},
    "openai": {"status": "up", "latency_ms": 120},
    "postgresql": {"status": "up", "latency_ms": 10}
  }
}
```

### Prometheus Metrics

Add Prometheus exporter to `main.py`:

```python
from prometheus_client import Counter, Histogram, make_asgi_app

# Metrics
request_count = Counter('api_requests_total', 'Total API requests', ['endpoint', 'status'])
request_duration = Histogram('api_request_duration_seconds', 'Request duration', ['endpoint'])
confidence_score = Histogram('confidence_score', 'Confidence scores', buckets=[0.5, 0.6, 0.7, 0.8, 0.9, 1.0])

# Mount metrics endpoint
metrics_app = make_asgi_app()
app.mount("/metrics", metrics_app)
```

**Prometheus scrape config:**
```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'rag-agent-api'
    static_configs:
      - targets: ['rag-agent-api-service:80']
    metrics_path: '/metrics'
    scrape_interval: 15s
```

### Grafana Dashboard

Key metrics to monitor:

1. **Request Rate**: Requests per second by endpoint
2. **Latency**: p50, p95, p99 response times
3. **Error Rate**: 4xx and 5xx errors
4. **Confidence Distribution**: Histogram of confidence scores
5. **Dependency Health**: Qdrant, OpenAI, PostgreSQL availability
6. **Resource Usage**: CPU, memory, database connections

### Logging

**Structured JSON Logging (Production):**

```python
# Example log entry
{
  "timestamp": "2025-12-17T10:30:00.000Z",
  "level": "INFO",
  "message": "Chat response generated",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "confidence_level": "high",
  "avg_similarity": 0.87,
  "num_chunks": 5,
  "execution_time_ms": 1250,
  "endpoint": "/chat/run"
}
```

**Log Aggregation:**
- ElasticStack (ELK): Elasticsearch + Logstash + Kibana
- Grafana Loki
- AWS CloudWatch
- GCP Cloud Logging

### Alerting

**Critical Alerts:**

1. **API Down**: Health check failures > 3 consecutive
2. **High Error Rate**: 5xx errors > 5% for 5 minutes
3. **Dependency Failure**: Qdrant/PostgreSQL unreachable
4. **High Latency**: p95 > 5 seconds for 10 minutes
5. **Low Confidence Rate**: >20% refusals for 30 minutes

**Example AlertManager rule:**

```yaml
groups:
- name: rag_agent_api
  rules:
  - alert: HighErrorRate
    expr: rate(api_requests_total{status=~"5.."}[5m]) > 0.05
    for: 5m
    labels:
      severity: critical
    annotations:
      summary: "High 5xx error rate"
      description: "Error rate is {{ $value | humanizePercentage }}"
```

---

## API Key Management & Rotation

### Generating API Keys

```python
# Generate secure API key
import secrets

api_key = secrets.token_urlsafe(32)
print(f"API Key: {api_key}")
```

### Storing API Keys

**Development:**
- Store in `.env` file (git-ignored)

**Production:**
- Use Kubernetes Secrets
- Use secret managers: AWS Secrets Manager, HashiCorp Vault
- Rotate every 90 days

### Key Rotation Procedure

1. **Generate new key:**
   ```bash
   NEW_KEY=$(python -c "import secrets; print(secrets.token_urlsafe(32))")
   echo $NEW_KEY
   ```

2. **Update Kubernetes secret:**
   ```bash
   kubectl create secret generic agent-api-secrets \
     --from-literal=API_KEY=$NEW_KEY \
     --namespace=rag-agent-api \
     --dry-run=client -o yaml | kubectl apply -f -
   ```

3. **Rolling restart:**
   ```bash
   kubectl rollout restart deployment/rag-agent-api -n rag-agent-api
   kubectl rollout status deployment/rag-agent-api -n rag-agent-api
   ```

4. **Notify clients** of new key (grace period: 7 days)

5. **Revoke old key** after grace period

### Multi-Key Support

For zero-downtime rotation, support multiple valid keys:

```python
# config.py
API_KEYS = os.getenv("API_KEYS", "").split(",")  # Comma-separated list
```

---

## Scaling & Performance

### Horizontal Scaling

**Kubernetes Autoscaling:**
- Min replicas: 3
- Max replicas: 10
- CPU target: 70%
- Memory target: 80%

**Manual Scaling:**
```bash
kubectl scale deployment/rag-agent-api --replicas=5 -n rag-agent-api
```

### Performance Tuning

**Database Connection Pool:**
```env
DB_POOL_SIZE=50           # Number of connections
DB_MAX_OVERFLOW=100       # Additional connections under load
```

**Agent Caching:**
```env
SESSION_EXPIRY_HOURS=2    # Cache agents longer in production
```

**Retrieval Optimization:**
```env
TOP_K_DEFAULT=3           # Fewer chunks = faster retrieval
```

### Load Testing

```bash
# Using locust
pip install locust

# Run load test
locust -f load_test.py --host https://api.example.com
```

**Expected Performance:**
- **Throughput**: 100 concurrent requests
- **Latency (p95)**: < 3 seconds
- **Latency (p99)**: < 5 seconds

---

## Troubleshooting

### Deployment Issues

**Pods not starting:**
```bash
kubectl describe pod <pod-name> -n rag-agent-api
kubectl logs <pod-name> -n rag-agent-api
```

**Common causes:**
- Missing secrets
- Invalid database URL
- Qdrant collection not found

**Health check failing:**
```bash
kubectl exec -it <pod-name> -n rag-agent-api -- curl localhost:8000/health
```

### Performance Issues

**High latency:**
- Check OpenAI API status
- Verify Qdrant response times
- Review database connection pool utilization
- Check if confidence thresholds are too low (causing many refusals)

**High error rate:**
- Check dependency health (Qdrant, PostgreSQL)
- Review OpenAI API quota
- Check rate limiting configuration

### Rollback Procedure

```bash
# View deployment history
kubectl rollout history deployment/rag-agent-api -n rag-agent-api

# Rollback to previous version
kubectl rollout undo deployment/rag-agent-api -n rag-agent-api

# Rollback to specific revision
kubectl rollout undo deployment/rag-agent-api --to-revision=2 -n rag-agent-api
```

---

## Deployment Checklist

Before deploying to production:

- [ ] All environment variables configured
- [ ] Secrets stored securely (not in code)
- [ ] Database initialized and backed up
- [ ] Qdrant collection populated
- [ ] Health checks passing
- [ ] Monitoring and alerting configured
- [ ] Log aggregation set up
- [ ] API keys rotated
- [ ] Load testing completed
- [ ] Rollback procedure tested
- [ ] Documentation updated
- [ ] Team trained on operations

---

**Last Updated**: 2025-12-17
