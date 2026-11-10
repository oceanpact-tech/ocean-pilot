OceanPilot

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

## Run MTX for video

```bash
docker compose -f compose.video-test.yaml up -d
```

## Run program

```bash
/root/app/build/bin/main
```
