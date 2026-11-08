# Preserve the already tested Unitree simulator environment.
# This is a small runtime layer, not a from-scratch simulator rebuild.
FROM g1-sim:dev

SHELL ["/bin/bash", "-c"]

RUN set -eux; \
    if ! getent group g1user >/dev/null; then groupadd -g 1002 g1user; fi; \
    if ! id -u g1user >/dev/null 2>&1; then \
        useradd -u 1002 -g g1user -d /tmp/g1user -s /bin/bash g1user; \
    fi; \
    mkdir -p /tmp/g1user; \
    chown g1user:g1user /tmp/g1user

ENV HOME=/tmp/g1user
WORKDIR /workspace
