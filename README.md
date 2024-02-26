# 3d-surface-planner

## 设置开发环境

本仓库推荐使用 Docker 进行开发和部署，如果你不熟悉 Docker，请查看博客 [使用 Docker 进行 ROS2 的开发](https://zhuanlan.zhihu.com/p/637040850)。

在进行以下操作之前，请确保你已经安装了 Docker 和 Nvidia Docker(如果你的电脑有 Nvidia 显卡)。

### 构建 Docker 镜像

```bash
cd .docker
./build.sh
```

此步骤会构建一个名为 `3d-surface-planner:latest` 的 Docker 镜像。

### 运行 Docker 容器

如果你的电脑有 Nvidia 显卡，则运行以下命令：

```bash
cd .docker
./run.sh
```

如果你的电脑没有 Nvidia 显卡，则运行以下命令：

```bash
cd .docker
./run.sh --no-nvidia
```

此步骤会在你的电脑上启动一个名为 `3d-surface-planner_latest` Docker 容器，该容器会在电脑重启后自动启动。

### 进入 Docker 容器

```bash
cd .docker
./join.sh
```

为了方便开发，推荐使用 `vscode` + `remote development` 插件直接进入开发容器。