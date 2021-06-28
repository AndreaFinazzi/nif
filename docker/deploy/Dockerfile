FROM ghcr.io/andreafinazzi/ade:ssh
RUN colcon build --base-paths src/* --merge-install --install-base /opt/nif --cmake-args -DCMAKE_BUILD_TYPE=Release


FROM  ubuntu:20.04
COPY --from=0 /opt/nif /opt/nif