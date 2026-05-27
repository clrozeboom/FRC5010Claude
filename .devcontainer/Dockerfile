FROM mcr.microsoft.com/devcontainers/java:17-bookworm

# xvfb provides a virtual display so the WPILib sim GUI (which tries to open
# a window) doesn't crash when you run `xvfb-run ./gradlew simulateJava`.
# libgl1-mesa-dev / libglu1-mesa are pulled in for OpenGL-based WPILib native libs.
# The base image may include a Yarn apt source with a missing signing key.
# Remove it so apt-get update can proceed using Debian repos.
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        libgl1-mesa-dev \
        libglu1-mesa \
        x11-utils \
        ca-certificates \
        curl \
    && rm -rf /var/lib/apt/lists/*

# uv (Astral) — provides `uvx` so MCP servers declared in .mcp.json (e.g. the
# `first-agentic-csa` FRC-docs server) can run inside the devcontainer / web
# sandbox without a separate Python toolchain install.
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh
