
## Create container

```bash
docker compose up -d
```

## Access container

```bash
docker exec -it ocean-pilot-gs bash
```

### Build image locally

```bash
docker compose -f docker/compose.build-image.yaml build --no-cache
```

