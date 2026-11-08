# Preserve the already tested Unitree simulator environment.
# This is a small runtime layer, not a from-scratch simulator rebuild.
FROM g1-sim:dev

SHELL ["/bin/bash", "-c"]

RUN set -eux; \
    if ! getent group helene >/dev/null; then groupadd -g 1002 helene; fi; \
    if ! id -u helene >/dev/null 2>&1; then \
        useradd -u 1002 -g helene -d /tmp/helene -s /bin/bash helene; \
    fi; \
    mkdir -p /tmp/helene; \
    chown helene:helene /tmp/helene

ENV HOME=/tmp/helene
WORKDIR /workspace
