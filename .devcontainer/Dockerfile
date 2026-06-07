FROM mcr.microsoft.com/devcontainers/java:17-bookworm

# xvfb provides a virtual display so the WPILib sim GUI (which tries to open
# a window) doesn't crash when you run `xvfb-run ./gradlew simulateJava`.
# libgl1-mesa-dev / libglu1-mesa are pulled in for OpenGL-based WPILib native libs.
# The base image may include a Yarn apt source with a missing signing key.
# Remove it so apt-get update can proceed using Debian repos.
#
# PhotonVision's 2026 native lib (libphotontargeting.so) is built against GCC 13+ and
# requires GLIBCXX_3.4.32 at load time. The bookworm base ships libstdc++6 from GCC 12
# (max GLIBCXX_3.4.30), so the PhotonVision JNI fails with UnsatisfiedLinkError and the
# vision tests can't run. Pull libstdc++6 (and its libc6 / libgcc-s1 deps) from Debian
# trixie to get 3.4.32+. trixie is pinned below bookworm so it is used ONLY for this
# explicit `-t trixie` install, never for the other packages above.
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && echo "deb http://deb.debian.org/debian trixie main" > /etc/apt/sources.list.d/trixie.list \
    && printf 'Package: *\nPin: release n=trixie\nPin-Priority: 100\n' > /etc/apt/preferences.d/99-trixie \
    && apt-get update && apt-get install -y --no-install-recommends \
        xvfb \
        libgl1-mesa-dev \
        libglu1-mesa \
        x11-utils \
        ca-certificates \
        curl \
    && apt-get install -y --no-install-recommends -t trixie libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

# uv (Astral) — provides `uvx` so MCP servers declared in .mcp.json (e.g. the
# `first-agentic-csa` FRC-docs server) can run inside the devcontainer / web
# sandbox without a separate Python toolchain install.
RUN curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR=/usr/local/bin sh
