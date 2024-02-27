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

此步骤会在你的电脑上启动一个名为 `3d-surface-planner_latest` Docker 容器。

### 进入 Docker 容器

```bash
cd .docker
./join.sh
```

为了方便开发，推荐使用 `vscode` + `remote development` 插件直接进入开发容器。

### 代码格式化

为了保证代码风格的一致性，本仓库使用 `clang-format` 对 C++ 进行格式化，使用 `flake8` 对 Python 进行格式化，使用 `xmllint` 对 xml 进行格式化。以上工具都已在 Dockerfile 中安装完毕。在 vscode 中需要安装以下插件： `clang-format`, `Flake8`, `XML` 并进行配置。为了保证代码风格的一致性，Docker 里安装了 `pre-commit` 工具在 `git commit` 之前对所提交的代码进行格式化。

在首次通过 `./join.sh` 进入 Docker 容器后，运行 `pre-commit install`以完成 `pre-commit` 的配置
