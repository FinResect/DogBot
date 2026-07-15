# DogBot Docker 构建命令

## 代理配置

宿主机代理地址：`http://172.17.0.1:7897/`（Docker bridge gateway）

## 一键构建（脚本）

```bash
./.script/docker-build-all
```

## 分步构建

### 1. 构建 dogbot-base:arm64

```bash
docker buildx build . --platform linux/arm64 --target dogbot-base -t dogbot-base:arm64 \
  --build-arg HTTP_PROXY=http://172.17.0.1:7897/ \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7897/ \
  --load
```

### 2. 构建 dogbot-develop-full

```bash
docker buildx build . --platform linux/amd64 --target dogbot-develop-full \
  --build-arg SYSROOT_IMAGE_ARM64=dogbot-base:arm64 \
  --build-arg HTTP_PROXY=http://172.17.0.1:7897/ \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7897/ \
  --load
```

### 3. 构建 dogbot-runtime:arm64

```bash
docker buildx build . --platform linux/arm64 --target dogbot-runtime -t dogbot-runtime:arm64 \
  --build-arg HTTP_PROXY=http://172.17.0.1:7897/ \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7897/ \
  --load
```

## 验证代理可用

```bash
docker run --rm -e https_proxy=http://172.17.0.1:7897/ alpine wget -q -O /dev/null -Y on -T 5 https://github.com && echo ok
```
