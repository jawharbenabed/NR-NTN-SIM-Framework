FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Paris

# ── Dépendances système ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential gcc g++ cmake ninja-build \
    python3 python3-dev python3-pip \
    libgsl-dev libsqlite3-dev libxml2-dev \
    libboost-all-dev libpcap-dev \
    git wget ca-certificates pkg-config \
    && rm -rf /var/lib/apt/lists/*

# ── Python (Hypatia) ─────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    numpy scipy pandas matplotlib networkx astropy ephem

# ── ns-3.46 ──────────────────────────────────────────────────────────────────
RUN git clone --depth 1 --branch ns-3.46 \
    https://gitlab.com/nsnam/ns-3-dev.git /opt/ns-3.46

# ── 5G-LENA v4.2 (module contrib/nr) ─────────────────────────────────────────
RUN git clone --depth 1 --branch v4.2 \
    https://gitlab.com/cttc-lena/nr.git /opt/ns-3.46/contrib/nr

# ── Hypatia (satgenpy + ns3-sat-sim) ─────────────────────────────────────────
RUN git clone --depth 1 \
    https://github.com/snkas/hypatia.git /opt/hypatia

# ── Modules Hypatia ns-3 (basic-sim + satellite-network) ─────────────────────
COPY basic-sim/         /opt/ns-3.46/contrib/basic-sim/
COPY satellite-network/ /opt/ns-3.46/contrib/satellite-network/

# ── Scripts de simulation NR-NTN-SIM ─────────────────────────────────────────
COPY ns3_nrntnsim_multiue.cc \
     ns3_nrntnsim_timeseries_optionD_approach5.cc \
     /opt/ns-3.46/scratch/

# ── Build ns-3 + 5G-LENA + modules Hypatia ───────────────────────────────────
RUN cd /opt/ns-3.46 && \
    ./ns3 configure --disable-tests --disable-examples -d optimized && \
    ./ns3 build

ENV NS3_HOME=/opt/ns-3.46 \
    HYPATIA_HOME=/opt/hypatia \
    PYTHONPATH=/opt/hypatia/satgenpy \
    PATH="/opt/ns-3.46:${PATH}"

WORKDIR /opt/ns-3.46
