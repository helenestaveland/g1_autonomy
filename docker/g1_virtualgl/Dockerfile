FROM g1-runtime:local

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl --fail --location --retry 3 \
       https://github.com/VirtualGL/virtualgl/releases/download/3.1.5/virtualgl_3.1.5_amd64.deb \
       --output /tmp/virtualgl.deb \
    && echo "df3f7788ce41b182a47c0d298e5cd6d2d63579522cb41825970b7726e825485e  /tmp/virtualgl.deb" \
       | sha256sum -c - \
    && apt-get install -y --no-install-recommends /tmp/virtualgl.deb \
    && rm -f /tmp/virtualgl.deb \
    && rm -rf /var/lib/apt/lists/*