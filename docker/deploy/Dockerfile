FROM ghcr.io/andreafinazzi/ade:master
RUN colcon build --base-paths src/* --merge-install --install-base /opt/nif --cmake-args -DCMAKE_BUILD_TYPE=Release


FROM  ros:foxy-ros-core
COPY --from=0 /opt/nif /opt/nif